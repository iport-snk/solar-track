#pragma once
#include <cstdint>
#include <string>
#include <stdexcept>
#include <chrono>
#include <mutex>
#include <cstring>
#include <string_view>
#include <charconv>
#include <tuple>
#include <iostream>
#include <thread>

#include <MQTTClient.h>
#include "State.hpp"
#include "Motors.hpp"
#include "Config.hpp"

class MqttClient {
public:
    static void init() {
        auto now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        std::lock_guard<std::mutex> lock(global_mutex_);
        if (initialized_) return;

        client_id_ = "graviton_" + std::to_string(now);
        initialized_ = true;

        int rc = MQTTClient_create(&handle_, CFG::mqttUri, client_id_.c_str(), MQTTCLIENT_PERSISTENCE_NONE, nullptr);
        if (rc != MQTTCLIENT_SUCCESS)
            throw std::runtime_error("Failed to create MQTT client");

        MQTTClient_setCallbacks(handle_, nullptr, nullptr,
            [](void* context, char* topicName, int topicLen, MQTTClient_message* message) -> int {
                std::string_view topic(topicName, topicLen > 0 ? topicLen : std::strlen(topicName));
                std::string_view payload(static_cast<char*>(message->payload), message->payloadlen);
                
                std::string resp = TrackerFSM::handleCmd(topic, payload);
                if (resp.size() > 0)  publish("response/cmd", resp);
                MQTTClient_freeMessage(&message);
                MQTTClient_free(topicName);
                return 1;
            },
            nullptr
        );

        connect();  // Initial connection attempt
        
        // Start reconnection thread
        keep_running_ = true;
        reconnect_thread_ = std::thread([]() {
            while (keep_running_) {
                std::this_thread::sleep_for(std::chrono::seconds(30));
                if (!connected_ && keep_running_) {
                    std::cout << "[MQTT] Attempting reconnection..." << std::endl;
                    connect();
                }
            }
        });
    }

    
    static void publish(const char* topic, std::string msg, int qos = 1, bool retained = false) {
        std::lock_guard<std::mutex> lock(global_mutex_);
        const char* payload = msg.c_str();

        MQTTClient_deliveryToken token;
        
        if (!connected_) {
            std::cout << "[MQTT] Cannot publish '" << topic << "': Not connected to broker" << std::endl;
            return;
        }

        int rc = MQTTClient_publish(handle_, topic, strlen(payload), payload, qos, retained, &token);
        if (rc != MQTTCLIENT_SUCCESS) {
            std::cout << "[MQTT] Failed to publish to '" << topic << "': " << getErrorString(rc) << std::endl;
            connected_ = false;  // Mark as disconnected to trigger reconnection
            return;
        }

        // MQTTClient_waitForCompletion(handle_, token, 1000);
    }

    static void shutdown() {
        keep_running_ = false;
        
        if (reconnect_thread_.joinable()) {
            reconnect_thread_.join();
        }
        
        std::lock_guard<std::mutex> lock(global_mutex_);
        if (connected_) {
            MQTTClient_disconnect(handle_, 1000);
            MQTTClient_destroy(&handle_);
            connected_ = false;
        }
    }

private:
    MqttClient() = default;

    static void connect() {
        MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
        MQTTClient_SSLOptions ssl_opts = MQTTClient_SSLOptions_initializer;
        ssl_opts.enableServerCertAuth = 0;
        ssl_opts.verify = 1;
        ssl_opts.CApath = NULL;
        ssl_opts.keyStore = NULL;
        ssl_opts.trustStore = NULL;
        ssl_opts.privateKey = NULL;
        ssl_opts.privateKeyPassword = NULL;
        ssl_opts.enabledCipherSuites = NULL;

        conn_opts.ssl = &ssl_opts;
        conn_opts.username = "rpi5solar";
        conn_opts.password = "Vc!Q4pf9Kku*WFW";
        conn_opts.connectTimeout = 5;
        conn_opts.keepAliveInterval = 20;
        conn_opts.cleansession = 1;

        int rc = MQTTClient_connect(handle_, &conn_opts);
        if (rc == MQTTCLIENT_SUCCESS) {
            connected_ = true;
            std::cout << "[MQTT] Connected successfully to broker" << std::endl;
            
            rc = MQTTClient_subscribe(handle_, "commander/#", 1);
            if (rc != MQTTCLIENT_SUCCESS) {
                std::cout << "[MQTT] Failed to subscribe to commander/#: " << getErrorString(rc) << std::endl;
                connected_ = false;
            } else {
                std::cout << "[MQTT] Subscribed to commander/# successfully" << std::endl;
            }
        } else {
            std::cout << "[MQTT] Connection failed: " << getErrorString(rc) << std::endl;
            connected_ = false;
        }
    }
    
    static const char* getErrorString(int rc) {
        switch (rc) {
            case MQTTCLIENT_SUCCESS: return "Success";
            case MQTTCLIENT_FAILURE: return "Generic failure";
            case MQTTCLIENT_PERSISTENCE_ERROR: return "Persistence error";
            case MQTTCLIENT_DISCONNECTED: return "Client disconnected";
            case MQTTCLIENT_MAX_MESSAGES_INFLIGHT: return "Max messages in-flight";
            case MQTTCLIENT_BAD_UTF8_STRING: return "Bad UTF8 string";
            case MQTTCLIENT_NULL_PARAMETER: return "Null parameter";
            case MQTTCLIENT_TOPICNAME_TRUNCATED: return "Topic name truncated";
            case MQTTCLIENT_BAD_STRUCTURE: return "Bad structure";
            case MQTTCLIENT_BAD_QOS: return "Bad QoS";
            case MQTTCLIENT_SSL_NOT_SUPPORTED: return "SSL not supported";
            case MQTTCLIENT_BAD_MQTT_VERSION: return "Bad MQTT version";
            case MQTTCLIENT_BAD_PROTOCOL: return "Bad protocol";
            case MQTTCLIENT_BAD_MQTT_OPTION: return "Bad MQTT option";
            case MQTTCLIENT_WRONG_MQTT_VERSION: return "Wrong MQTT version";
            default: return "Unknown error";
        }
    }

    static inline std::string client_id_;
    static inline MQTTClient handle_ = nullptr;
    static inline bool initialized_ = false;
    static inline bool connected_ = false;
    static inline bool keep_running_ = false;
    static inline std::mutex global_mutex_;
    static inline std::thread reconnect_thread_;
};