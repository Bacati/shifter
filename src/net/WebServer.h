#pragma once
#include <Arduino.h>

// ============================================================================
//  WebServer : routes HTTP + SSE + authentification.
//
//   GET  /              page statique (aucune substitution serveur)
//   GET  /api/state     JSON : config + tier + ssid + hwid + flags
//   GET  /events        flux SSE de telemetrie
//   POST /api/login     pin -> jeton Bearer (TTL RAM) ; lockout apres echecs
//   POST /api/config    [auth] ecrit la carto (champs TRACK/PRO gates)
//   POST /api/license   [rate-limite] deblocage de licence
//   POST /api/wifi      [auth] rotation du mot de passe AP
//   POST /api/ota       [auth] mise a jour firmware (refusee moteur tournant)
// ============================================================================

namespace web {

void   begin();
size_t sseClients();
void   sseSend(const char* json, const char* event);

}  // namespace web
