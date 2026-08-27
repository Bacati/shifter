#pragma once
// ============================================================================
//  MODELE DE SECRET  -  NE PAS COMMITER LE FICHIER "secrets.h" REEL
// ============================================================================
//
//  1. Copiez ce fichier :   src/persistence/secrets.example.h
//                      ->   src/persistence/secrets.h
//  2. Remplacez la valeur ci-dessous par une chaine longue et unique
//     (>= 32 caracteres aleatoires).
//  3. "secrets.h" est ignore par git (.gitignore).
//
//  Ce secret sert de cle HMAC-SHA256 pour generer / verifier les cles de
//  licence liees au HWID (adresse MAC) de chaque calculateur.
//
//  RAPPEL SECURITE : sur une puce sans Flash Encryption + Secure Boot v2,
//  ce secret reste extractible par dump de la flash SPI. Le DRM ne fait que
//  hausser la barre ; il n'est pas inviolable. Voir BUILD.md.
// ============================================================================

#define LICENSE_HMAC_SECRET "CHANGE-MOI-secret-unique-et-long-0123456789"
