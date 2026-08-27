#pragma once

// ============================================================================
//  Factory : console serie de provisioning (lecture MAC, injection de licence).
//
//  Active UNIQUEMENT avec -DFACTORY_TOOLS (environnement [env:factory]).
//  Sinon : no-op total, rien n'est embarque dans le firmware de prod.
//
//  Protocole (une commande par ligne, reponses prefixees pour parsing) :
//    MAC?            -> "MAC AA:BB:CC:DD:EE:FF"
//    INFO            -> "INFO fw=.. mac=.. tier=n(NOM)"
//    KEY <cle>       -> "OK tier=n"  |  "FAIL bad_key"  |  "LOCKED retry_ms=.."
//    WIPE            -> efface la licence NVS puis reboot
// ============================================================================

namespace factory {
void begin();  // banniere + MAC au boot
void poll();   // a appeler dans loop()
}
