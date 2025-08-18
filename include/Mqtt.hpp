#pragma once
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
#include "GPIO.hpp"

constexpr auto parseCommand = [](std::string_view input) {
    size_t p1 = input.find('|');
    size_t p2 = (p1 != std::string_view::npos) ? input.find('|', p1 + 1) : std::string_view::npos;

    std::string_view cmd;
    float f1 = 0.0f, f2 = 0.0f;

    if (p1 == std::string_view::npos) {
        cmd = input;
    } else {
        cmd = input.substr(0, p1);
        std::string_view s1 = input.substr(p1 + 1, (p2 != std::string_view::npos ? p2 : input.size()) - p1 - 1);
        std::from_chars(s1.data(), s1.data() + s1.size(), f1);

        if (p2 != std::string_view::npos) {
            std::string_view s2 = input.substr(p2 + 1);
            std::from_chars(s2.data(), s2.data() + s2.size(), f2);
        }
    }

    return std::tuple{cmd, f1, f2};
};


class MqttClient {
public:
    static void init(const std::string& host, int port) {
        auto now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        std::lock_guard<std::mutex> lock(global_mutex_);
        if (initialized_) return;

        uri_ = host + ":" + std::to_string(port);
        client_id_ = "graviton_" + std::to_string(now);
        initialized_ = true;

        int rc = MQTTClient_create(&handle_, uri_.c_str(), client_id_.c_str(), MQTTCLIENT_PERSISTENCE_NONE, nullptr);
        if (rc != MQTTCLIENT_SUCCESS)
            throw std::runtime_error("Failed to create MQTT client");

        MQTTClient_setCallbacks(handle_, nullptr, nullptr,
            [](void* context, char* topicName, int topicLen, MQTTClient_message* message) -> int {
                std::string topic(topicName, topicLen > 0 ? topicLen : std::strlen(topicName));
                std::string payload(static_cast<char*>(message->payload), message->payloadlen);
                auto [cmd, p1, p2] = parseCommand(payload);

                std::cout << "[COMMAND] " << topic << " → " << payload << "\n";
                // TODO: Parse and act on payload
                if (            cmd == "move"   ) { TrackerFSM::handleMoveTo(p1, p2); } 
                else if (       cmd == "stop"   ) { TrackerFSM::handleStop(); } 
                else if (       cmd == "auto"   ) { TrackerFSM::handleAuto(); } 
                else if (       cmd == "mu"     ) { GPIO::moveU(); } 
                else if (       cmd == "md"     ) { GPIO::moveD(); } 
                else if (       cmd == "me"     ) { GPIO::moveE(); } 
                else if (       cmd == "mw"     ) { GPIO::moveW(); }
                else if (       cmd == "sel"    ) { GPIO::stopEl(); }
                else if (       cmd == "saz"    ) { GPIO::stopAz(); };
                

                MQTTClient_freeMessage(&message);
                MQTTClient_free(topicName);
                return 1;
            },
            nullptr
        );

        connect();  // Initial connection
    }

    
    static void publish(const std::string& topic, const std::string& payload, int qos = 1, bool retained = false) {
        std::lock_guard<std::mutex> lock(global_mutex_);
        if (!connected_) connect();  // Silent reconnect

        MQTTClient_message pubmsg = MQTTClient_message_initializer;
        pubmsg.payload = const_cast<char*>(payload.c_str());
        pubmsg.payloadlen = static_cast<int>(payload.size());
        pubmsg.qos = qos;
        pubmsg.retained = retained;

        MQTTClient_deliveryToken token;
        int rc = MQTTClient_publishMessage(handle_, topic.c_str(), &pubmsg, &token);
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

    static inline std::string uri_;
    static inline std::string client_id_;
    static inline MQTTClient handle_ = nullptr;
    static inline bool initialized_ = false;
    static inline bool connected_ = false;
    static inline std::mutex global_mutex_;
};