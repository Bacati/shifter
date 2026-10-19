#pragma once
// Faux ESPAsyncWebServer : enregistre les routes et laisse les tests appeler les gestionnaires
// du VRAI WebServer.cpp avec des requetes fabriquees.
#include "Arduino.h"
#include <functional>
#include <map>
#include <string>

#define HTTP_GET 1
#define HTTP_POST 2

class AsyncWebServerResponse {
 public:
  int code = 0; std::string type, body; std::map<std::string, std::string> headers;
  void addHeader(const char* k, const String& v) { headers[k] = v.c_str(); }
};
class AsyncWebParameter {
  String v_;
 public:
  explicit AsyncWebParameter(const String& v) : v_(v) {}
  String value() const { return v_; }
};
class AsyncWebServerRequest {
 public:
  std::map<std::string, std::string> post, hdr;
  AsyncWebServerResponse* sent = nullptr;
  bool hasParam(const char* n, bool isPost = false) const { return isPost && post.count(n); }
  AsyncWebParameter* getParam(const char* n, bool = false) const {
    static AsyncWebParameter* last = nullptr; delete last; last = new AsyncWebParameter(String(post.at(n))); return last;
  }
  bool hasHeader(const char* n) const { return hdr.count(n) != 0; }
  String header(const char* n) const { auto it = hdr.find(n); return it == hdr.end() ? String() : String(it->second); }
  AsyncWebServerResponse* beginResponse(int code, const char* type, const String& body) {
    auto* r = new AsyncWebServerResponse(); r->code = code; r->type = type; r->body = body.c_str(); return r;
  }
  AsyncWebServerResponse* beginResponse(int code, const char* type, const uint8_t* d, size_t n) {
    auto* r = new AsyncWebServerResponse(); r->code = code; r->type = type; r->body.assign((const char*)d, n); return r;
  }
  void send(AsyncWebServerResponse* r) { sent = r; }
  void send(int code, const char* type, const String& body) { sent = beginResponse(code, type, body); }
};
typedef void (*ArRequestHandlerFunction)(AsyncWebServerRequest*);
typedef void (*ArUploadHandlerFunction)(AsyncWebServerRequest*, const String&, size_t, uint8_t*, size_t, bool);

namespace hostweb {
struct Route { ArRequestHandlerFunction h; ArUploadHandlerFunction up; };
inline std::map<std::string, Route>& routes() { static std::map<std::string, Route> r; return r; }
inline size_t& sseCount() { static size_t n = 0; return n; }
}  // namespace hostweb

class AsyncEventSource {
 public:
  explicit AsyncEventSource(const char*) {}
  size_t count() const { return hostweb::sseCount(); }
  void send(const char*, const char*, uint32_t) {}
};
class AsyncWebServer {
 public:
  explicit AsyncWebServer(int) {}
  void on(const char* p, int, ArRequestHandlerFunction h, ArUploadHandlerFunction up = nullptr) { hostweb::routes()[p] = {h, up}; }
  void addHandler(AsyncEventSource*) {}
  void onNotFound(std::function<void(AsyncWebServerRequest*)>) {}
  void begin() {}
};
