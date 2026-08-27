#pragma once

// ============================================================================
//  Telemetry : taches Core 0.
//   - sseTask        : pousse la telemetrie SSE (JSON) aux clients connectes.
//   - supervisorTask : surveille le heartbeat de loop() ; si fige, rend
//                      l'etincelle (ignition::forceClear) puis redemarre.
// ============================================================================

namespace telemetry {
void startTasks();
}
