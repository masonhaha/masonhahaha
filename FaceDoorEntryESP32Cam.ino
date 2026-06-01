#include <ArduinoWebsockets.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "camera_index.h"
#include "Arduino.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include "fr_flash.h"
#include "SD_MMC.h"

using namespace websockets;
WebsocketsServer socket_server;
camera_fb_t * fb = NULL;
long current_millis;
long last_detected_millis = 0;

#define relay_pin 2
#define interval 5000
bool face_recognised = false;
long door_opened_millis = 0;

// ================= WiFi 配置 (请修改为你自己的 WiFi 信息) =================
const char* ssid = "FAST_shihome";
const char* password = "shiyaojun110";

#define ENROLL_CONFIRM_TIMES 5
#define FACE_ID_SAVE_NUMBER 7
#define ENROLL_NAME_LEN 32

// AI-THINKER ESP32-CAM 摄像头引脚定义
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ================= 全局变量 =================
int photoNumber = 0;
void app_facenet_main();
void app_httpserver_init();

httpd_handle_t camera_httpd = NULL;

void savePhotoToSD(camera_fb_t *fb);
#define MAX_PHOTO_COUNT 100 // 保留最近100张照片

typedef struct {
    uint8_t *image;
    box_array_t *net_boxes;
    dl_matrix3d_t *face_id;
} http_img_process_result;

static inline mtmn_config_t app_mtmn_config() {
    mtmn_config_t mtmn_config = {0};
    mtmn_config.type = FAST;
    mtmn_config.min_face = 80;
    mtmn_config.pyramid = 0.707;
    mtmn_config.pyramid_times = 4;
    mtmn_config.p_threshold.score = 0.6;
    mtmn_config.p_threshold.nms = 0.7;
    mtmn_config.p_threshold.candidate_number = 20;
    mtmn_config.r_threshold.score = 0.7;
    mtmn_config.r_threshold.nms = 0.7;
    mtmn_config.r_threshold.candidate_number = 10;
    mtmn_config.o_threshold.score = 0.7;
    mtmn_config.o_threshold.nms = 0.7;
    mtmn_config.o_threshold.candidate_number = 1;
    return mtmn_config;
}

mtmn_config_t mtmn_config = app_mtmn_config();
face_id_name_list st_face_list;
static dl_matrix3du_t *aligned_face = NULL;

typedef enum {
    START_STREAM,
    START_DETECT,
    SHOW_FACES,
    START_RECOGNITION,
    START_ENROLL,
    ENROLL_COMPLETE,
    DELETE_ALL,
} en_fsm_state;

en_fsm_state g_state;

typedef struct {
    char enroll_name[ENROLL_NAME_LEN];
} httpd_resp_value;

httpd_resp_value st_name;

// ================= HTTP 处理函数定义 =================

// 1. 跨域预检处理函数 (OPTIONS)
static esp_err_t options_handler(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

// 2. 文件列表处理函数 (已优化：增加连接状态检查，防止底层 setsockopt 报错)
static esp_err_t list_files_handler(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    httpd_resp_set_type(req, "application/json");
    
    std::string response = "[";
    File root = SD_MMC.open("/sdcard");
    if (root) {
        File file = root.openNextFile();
        bool first = true;
        while (file) {
            if (!file.isDirectory()) {
                if (!first) response.append(",");
                response.append("\"").append(String(file.name()).substring(1).c_str()).append("\"");
                first = false;
            }
            file = root.openNextFile();
            // 优化点：定期检查客户端是否还连着，防止向已断开的连接发送数据
            if (httpd_req_get_hdr_value_len(req, "Host") == 0) {
                return ESP_FAIL; 
            }
        }
        root.close();
    }
    response.append("]");

    esp_err_t res = httpd_resp_send(req, response.c_str(), HTTPD_RESP_USE_STRLEN);
    if (res != ESP_OK) return ESP_FAIL; // 发送失败静默处理
    return ESP_OK;
}

// 3. 文件读取处理函数 (已优化：分块发送时检查连接状态)
static esp_err_t get_file_handler(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");

    char filename[64];
    if (httpd_req_get_url_query_str(req, filename, sizeof(filename)) == ESP_OK) {
        char fname[64];
        char *param_start = strstr(filename, "filename=");
        if (param_start) {
            sscanf(param_start + 9, "%[^&]", fname);
            
            String path = "/sdcard/";
            path += fname;

            File file = SD_MMC.open(path.c_str());
            if (!file) {
                httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File Not Found");
                return ESP_OK;
            }

            if (String(fname).endsWith(".jpg")) {
                httpd_resp_set_type(req, "image/jpeg");
            } else {
                httpd_resp_set_type(req, "application/octet-stream");
            }

            httpd_resp_set_hdr(req, "Content-Disposition", "inline");
            
            size_t totalSent = 0;
            uint8_t buffer[1024]; 
            esp_err_t res = ESP_OK;
            
            while (file.available() && totalSent < file.size()) {
                // 优化点：每次发送前检查客户端是否已经断开连接
                if (httpd_req_get_hdr_value_len(req, "Host") == 0) {
                    file.close();
                    return ESP_FAIL;
                }

                int len = file.read(buffer, (sizeof(buffer) < (size_t)file.available()) ? sizeof(buffer) : (size_t)file.available());
                if (len > 0) {
                    res = httpd_resp_send_chunk(req, (const char*)buffer, len);
                    if (res != ESP_OK) break; // 如果发送失败，立刻停止发送
                    totalSent += len;
                }
            }
            file.close();
            
            if (res == ESP_OK) {
                httpd_resp_send_chunk(req, NULL, 0); // 发送结束标志
            }
        } else {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Filename Required");
        }
    } else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Query String Required");
    }
    return ESP_OK;
}

// 4. 首页处理函数
static esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len);
}

// ================= URI 结构体定义 =================
httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL
};

// ================= 服务器初始化 =================
void app_httpserver_init() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 7; 
    
    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        Serial.println("HTTP Server Started");
        
        httpd_register_uri_handler(camera_httpd, &index_uri);

        // 注册文件列表接口
        httpd_uri_t list_uri = {
            .uri = "/listfiles",
            .method = HTTP_GET,
            .handler = list_files_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(camera_httpd, &list_uri);
        
        // 注册文件列表跨域预检接口
        httpd_uri_t list_options_uri = {
            .uri = "/listfiles",
            .method = HTTP_OPTIONS,
            .handler = options_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(camera_httpd, &list_options_uri);

        // 注册文件读取接口
        httpd_uri_t getfile_uri = {
            .uri = "/getfile",
            .method = HTTP_GET,
            .handler = get_file_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(camera_httpd, &getfile_uri);
        
        // 注册文件读取跨域预检接口
        httpd_uri_t getfile_options_uri = {
            .uri = "/getfile",
            .method = HTTP_OPTIONS,
            .handler = options_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(camera_httpd, &getfile_options_uri);
    }
}

// ================= 人脸识别与业务逻辑 =================
void app_facenet_main() {
    face_id_name_init(&st_face_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);
    aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
    read_face_id_from_flash_with_name(&st_face_list);
}

static inline int do_enrollment(face_id_name_list *face_list, dl_matrix3d_t *new_id) {
    ESP_LOGD(TAG, "START ENROLLING");
    int left_sample_face = enroll_face_id_to_flash_with_name(face_list, new_id, st_name.enroll_name);
    ESP_LOGD(TAG, "Face ID %s Enrollment: Sample %d", st_name.enroll_name, ENROLL_CONFIRM_TIMES - left_sample_face);
    return left_sample_face;
}

static esp_err_t send_face_list(WebsocketsClient &client) {
    client.send("delete_faces");
    face_id_node *head = st_face_list.head;
    char add_face[64];
    for (int i = 0; i < st_face_list.count; i++) {
        sprintf(add_face, "listface:%s", head->id_name);
        client.send(add_face);
        head = head->next;
    }
}

static esp_err_t delete_all_faces(WebsocketsClient &client) {
    delete_face_all_in_flash_with_name(&st_face_list);
    client.send("delete_faces");
}

void handle_message(WebsocketsClient &client, WebsocketsMessage msg) {
    if (msg.data() == "stream") {
        g_state = START_STREAM;
        client.send("STREAMING");
    }
    if (msg.data() == "detect") {
        g_state = START_DETECT;
        client.send("DETECTING");
    }
    if (msg.data().substring(0, 8) == "capture:") {
        g_state = START_ENROLL;
        char person[FACE_ID_SAVE_NUMBER * ENROLL_NAME_LEN] = {0,};
        msg.data().substring(8).toCharArray(person, sizeof(person));
        memcpy(st_name.enroll_name, person, strlen(person) + 1);
        client.send("CAPTURING");
    }
    if (msg.data() == "recognise") {
        g_state = START_RECOGNITION;
        client.send("RECOGNISING");
    }
    if (msg.data().substring(0, 7) == "remove:") {
        char person[ENROLL_NAME_LEN * FACE_ID_SAVE_NUMBER];
        msg.data().substring(7).toCharArray(person, sizeof(person));
        delete_face_id_in_flash_with_name(&st_face_list, person);
        send_face_list(client);
    }
    if (msg.data() == "delete_all") {
        delete_all_faces(client);
    }
}

void open_door(WebsocketsClient &client) {
    if (digitalRead(relay_pin) == LOW) {
        digitalWrite(relay_pin, HIGH);
        Serial.println("Door Unlocked");
        client.send("door_open");
        door_opened_millis = millis();
    }
}

void savePhotoToSD(camera_fb_t *fb) {
    int current_index = photoNumber % MAX_PHOTO_COUNT;
    char filename[64];
    sprintf(filename, "/sdcard/photo%03d.jpg", current_index);
    
    File file = SD_MMC.open(filename, FILE_WRITE);
    if (!file) {
        Serial.println("❌ 打开文件失败: " + String(filename));
        return;
    }
    file.write(fb->buf, fb->len);
    file.close();
    Serial.println("📸 照片已保存/覆盖: " + String(filename));
    
    photoNumber++;
    if (current_index == 0 && photoNumber > MAX_PHOTO_COUNT) {
        Serial.println("🔄 照片存储已循环，开始覆盖最旧的照片.");
    }
}

// ================= 主程序 Setup 与 Loop =================
void setup() {
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    Serial.println();
    digitalWrite(relay_pin, LOW);
    pinMode(relay_pin, OUTPUT);

    // SD_MMC 初始化
    Serial.println("正在尝试挂载 SD卡...");
    if (!SD_MMC.begin("/sdcard", true)) {
        Serial.println("❌ SD卡挂载失败！请检查接线或SD卡格式(FAT32)。");
    } else {
        Serial.println("✅ SD卡挂载成功！");
        uint8_t cardType = SD_MMC.cardType();
        if (cardType == CARD_NONE) {
            Serial.println("未检测到 SD 卡");
        } else {
            Serial.print("SD 卡类型: ");
            if (cardType == CARD_MMC) Serial.println("MMC");
            else if (cardType == CARD_SD) Serial.println("SDSC");
            else if (cardType == CARD_SDHC) Serial.println("SDHC");
            else Serial.println("UNKNOWN");
            Serial.printf("SD 卡总容量: %llu MB\n", SD_MMC.cardSize() / (1024 * 1024));
        }
    }

    // 摄像头初始化
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    
    if (psramFound()) {
        config.frame_size = FRAMESIZE_UXGA;
        config.jpeg_quality = 5;
        config.fb_count = 2;
    } else {
        config.frame_size = FRAMESIZE_SVGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }
    sensor_t * s = esp_camera_sensor_get();
    s->set_framesize(s, FRAMESIZE_QVGA);

    // 连接 WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");

    // 初始化人脸识别与 HTTP 服务器
    app_facenet_main();
    app_httpserver_init();

    socket_server.listen(82);
    Serial.print("Camera Ready! Use 'http://");
    Serial.print(WiFi.localIP());
    Serial.println("' to connect");
}

void loop() {
    auto client = socket_server.accept();
    client.onMessage(handle_message);
    
    dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, 320, 240, 3);
    http_img_process_result out_res = {0};
    out_res.image = image_matrix->item;
    
    send_face_list(client);
    client.send("STREAMING");
    
    while (client.available()) {
        client.poll();
        
        if (millis() - interval > door_opened_millis) {
            digitalWrite(relay_pin, LOW);
        }
        
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Camera capture failed");
            continue;
        }
        
        if (g_state == START_DETECT || g_state == START_ENROLL || g_state == START_RECOGNITION) {
            out_res.net_boxes = NULL;
            out_res.face_id = NULL;
            fmt2rgb888(fb->buf, fb->len, fb->format, out_res.image);
            out_res.net_boxes = face_detect(image_matrix, &mtmn_config);
            
            if (out_res.net_boxes) {
                if (align_face(out_res.net_boxes, image_matrix, aligned_face) == ESP_OK) {
                    out_res.face_id = get_face_id(aligned_face);
                    last_detected_millis = millis();
                    
                    if (g_state == START_DETECT) {
                        client.send("FACE DETECTED");
                    }
                    
                    if (g_state == START_ENROLL) {
                        int left_sample_face = do_enrollment(&st_face_list, out_res.face_id);
                        char enrolling_message[64];
                        sprintf(enrolling_message, "SAMPLE NUMBER %d FOR %s", ENROLL_CONFIRM_TIMES - left_sample_face, st_name.enroll_name);
                        client.send(enrolling_message);
                        
                        if (left_sample_face == 0) {
                            ESP_LOGI(TAG, "Enrolled Face ID: %s", st_name.enroll_name);
                            g_state = START_STREAM;
                            char captured_message[64];
                            sprintf(captured_message, "FACE CAPTURED FOR %s", st_name.enroll_name);
                            client.send(captured_message);
                            send_face_list(client);
                        }
                    }
                    
                    if (g_state == START_RECOGNITION && (st_face_list.count > 0)) {
                        face_id_node *f = recognize_face_with_name(&st_face_list, out_res.face_id);
                        if (f) {
                            char recognised_message[64];
                            sprintf(recognised_message, "DOOR OPEN FOR %s", f->id_name);
                            open_door(client);
                            client.send(recognised_message);
                            savePhotoToSD(fb);
                        } else {
                            client.send("FACE NOT RECOGNISED");
                        }
                        dl_matrix3d_free(out_res.face_id);
                    }
                }
            } else {
                if (g_state != START_DETECT) {
                    client.send("NO FACE DETECTED");
                }
            }
            if (g_state == START_DETECT && millis() - last_detected_millis > 500) {
                client.send("DETECTING");
            }
        }
        
        if (fb) {
            client.sendBinary((const char *)fb->buf, fb->len);
            esp_camera_fb_return(fb);
            fb = NULL;
        }
    }
    
    dl_matrix3du_free(image_matrix);
}