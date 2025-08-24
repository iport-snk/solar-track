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

        connect();  // Initial connection
    }

    
    static void publish(const char* topic, std::string msg, int qos = 1, bool retained = false) {
        std::lock_guard<std::mutex> lock(global_mutex_);
        const char* payload = msg.c_str();

        MQTTClient_deliveryToken token;
        
        int rc = MQTTClient_publish(handle_, topic, strlen(payload), payload, qos, retained, &token);
        if (rc != MQTTCLIENT_SUCCESS) return;  // Silently fail

        // MQTTClient_waitForCompletion(handle_, token, 1000);
    }

    static void shutdown() {
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
            rc = MQTTClient_subscribe(handle_, "commander/#", 1);
            if (rc != MQTTCLIENT_SUCCESS) throw std::runtime_error("Failed to subscribe in init()");
        }
    }

    static inline std::string client_id_;
    static inline MQTTClient handle_ = nullptr;
    static inline bool initialized_ = false;
    static inline bool connected_ = false;
    static inline std::mutex global_mutex_;
};