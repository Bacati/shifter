#pragma once
#include <Arduino.h>
#include <pgmspace.h>

// ============================================================================
//  Page web embarquee (statique). Aucune substitution serveur : la config est
//  recuperee par le client via GET /api/state. Les mutations passent par les
//  endpoints /api/* avec un jeton Bearer (voir WebServer.cpp).
// ============================================================================

static const char INDEX_HTML[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html><html lang="fr"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>TTP ECU</title>
<style>
  body{font-family:Arial,Helvetica,sans-serif;background:#111;color:#eee;margin:0;padding:18px 12px;text-align:center}
  h1{color:#f39c12;margin:0 0 4px}
  h3{margin:0 0 10px;color:#ddd}
  .tier{display:inline-block;background:#8e44ad;padding:3px 10px;border-radius:5px;font-size:12px;margin-bottom:14px}
  .card{background:#1e1e1e;border:1px solid #333;padding:16px;border-radius:10px;max-width:420px;margin:0 auto 16px;text-align:left}
  .row{display:flex;justify-content:space-between;border-top:1px solid #333;margin-top:10px;padding-top:10px;text-align:center}
  .v{font-size:24px;font-weight:bold;color:#0fc}
  .v.warn{color:#e74c3c}
  .badge{display:inline-block;background:#555;padding:5px 12px;border-radius:20px;font-weight:bold;font-size:13px}
  .badge.mPITLIMIT,.badge.mLAUNCH{background:#e74c3c}
  .badge.mSHIFTING{background:#3498db}
  .badge.mSLIPCTL{background:#f1c40f;color:#000}
  .badge.mOVERREV{background:#8b0000}
  label{display:block;margin-top:12px;font-size:13px;font-weight:bold}
  input{width:100%;padding:9px;margin-top:4px;border:none;border-radius:5px;box-sizing:border-box;background:#eee;font-weight:bold}
  input:disabled{background:#444;color:#888}
  button{width:100%;margin-top:14px;padding:13px;border:none;border-radius:5px;background:#f39c12;color:#fff;font-size:15px;font-weight:bold;cursor:pointer}
  button.alt{background:#27ae60}
  .hint{font-size:12px;color:#9aa;margin:6px 0 0}
  .tag{background:#8e44ad;border-radius:4px;padding:1px 5px;font-size:10px}
  progress{width:100%;margin-top:10px}
</style>
</head><body>
<h1>TTP ECU</h1>
<div class="tier" id="tier">Licence: &mdash;</div>

<div class="card">
  <h3>Telemetrie</h3>
  <div class="badge" id="mode">IDLE</div>
  <div class="row">
    <div><small>RPM</small><div class="v" id="rpm">0</div></div>
    <div><small>Vitesse</small><div class="v" id="spd">0</div><small>km/h</small></div>
    <div><small>Slip</small><div class="v" id="slip">0</div><small>%</small></div>
    <div><small>Roulis</small><div class="v" id="lean">0</div><small>&deg;</small></div>
    <div><small>Eau</small><div class="v" id="waterTemp">&mdash;</div><small>&deg;C</small></div>
  </div>
  <p class="hint" id="imuState">IMU : &mdash;</p>
  <p class="hint" id="waterState">Capteur eau : &mdash;</p>
</div>

<div class="card">
  <h3>Acces configuration</h3>
  <p class="hint">PIN affiche sur le port serie au demarrage.</p>
  <input id="pin" type="password" inputmode="numeric" maxlength="6" placeholder="PIN a 6 chiffres">
  <button id="btnLogin">Se connecter</button>
  <p class="hint" id="authState">Non authentifie</p>
</div>

<div class="card">
  <h3>Cartographie &amp; limites</h3>
  <label>Cut shifter (ms)<input id="shiftCutMs" type="number"></label>
  <label>RPM min shifter<input id="minShiftRpm" type="number"></label>
  <label>Pit limiter : vitesse max (km/h)<input id="pitKmh" type="number"></label>
  <label>Pit limiter : repli regime (RPM)<input id="pitRpm" type="number"></label>
  <p class="hint">Sans capteurs de roue (licence BASIC) la vitesse est inconnue : le limiteur utilise ce regime.
  Calcul pour la vitesse voulue, dans le rapport engage en voie des stands (circonference de roue ci-dessous).</p>
  <label>Rapport total moteur/roue (tours moteur pour 1 tour de roue)<input id="pitRatio" type="number" step="0.01" placeholder="ex. 9.50"></label>
  <button id="btnPitCalc" class="alt">Calculer le regime pour cette vitesse</button>
  <p class="hint" id="pitCalcOut"></p>
  <label>Circonference roue (mm)<input id="wheelCircumMm" type="number"></label>
  <label>Impulsions / tour<input id="pulsesPerRev" type="number"></label>
  <label>Launch RPM <span class="tag">TRACK</span><input id="launchRpm" type="number"></label>
  <label>Tolerance slip (%) <span class="tag">PRO</span><input id="maxSlipPercent" type="number"></label>
  <button id="btnSave">Flasher NVRAM</button>
</div>

<div class="card">
  <h3>Licence</h3>
  <p class="hint">HWID : <span id="hwid">&mdash;</span></p>
  <input id="key" type="text" maxlength="16" placeholder="Cle de licence">
  <button id="btnKey" class="alt">Activer</button>
</div>

<div class="card">
  <h3>WiFi</h3>
  <p class="hint">SSID : <span id="ssid">&mdash;</span></p>
  <button id="btnWifi">Nouveau mot de passe AP</button>
  <p class="hint" id="wifiState"></p>
</div>

<div class="card">
  <h3>Mise a jour firmware (OTA)</h3>
  <p class="hint">Version : <span id="fw">&mdash;</span> &middot; moteur a l'arret obligatoire.</p>
  <input id="fwfile" type="file" accept=".bin">
  <button id="btnOta">Envoyer</button>
  <progress id="otaProg" value="0" max="100" hidden></progress>
  <p class="hint" id="otaState"></p>
</div>

<script>
(function(){
  var $=function(id){return document.getElementById(id);};
  var FIELDS=['shiftCutMs','minShiftRpm','pitRpm','pitKmh','wheelCircumMm','pulsesPerRev','launchRpm','maxSlipPercent'];
  var TOKEN='';
  try{TOKEN=sessionStorage.getItem('ecu_token')||'';}catch(e){}

  function setAuth(t){
    TOKEN=t||'';
    try{ if(t) sessionStorage.setItem('ecu_token',t); else sessionStorage.removeItem('ecu_token'); }catch(e){}
    $('authState').textContent=t?'Authentifie':'Non authentifie';
  }
  function authHeader(){return TOKEN?{'Authorization':'Bearer '+TOKEN}:{};}
  function form(obj){return Object.keys(obj).map(function(k){return k+'='+encodeURIComponent(obj[k]);}).join('&');}
  setAuth(TOKEN);

  function loadState(){
    fetch('/api/state').then(function(r){return r.json();}).then(function(d){
      $('fw').textContent=d.fw; $('ssid').textContent=d.ssid; $('hwid').textContent=d.hwid;
      $('tier').textContent='Licence: '+d.tierName;
      FIELDS.forEach(function(k){ if($(k)&&d[k]!=null) $(k).value=d[k]; });
      $('launchRpm').disabled=!d.trackUnlocked;
      $('maxSlipPercent').disabled=!d.proUnlocked;
    }).catch(function(){});
  }
  // Regime moteur correspondant a une vitesse, dans un rapport donne (moto sans capteurs de roue).
  function pitRpmFor(kmh,circMm,ratio){ return Math.round(kmh/3.6/(circMm/1000)*60*ratio); }
  $('btnPitCalc').addEventListener('click',function(){
    var kmh=+$('pitKmh').value, circ=+$('wheelCircumMm').value, ratio=+$('pitRatio').value, o=$('pitCalcOut');
    if(!(kmh>0&&circ>0&&ratio>0)){ o.textContent='Renseignez vitesse, circonference et rapport.'; return; }
    var rpm=pitRpmFor(kmh,circ,ratio);
    if(rpm<2000||rpm>10000){ o.textContent=rpm+' tr/min : hors de la plage 2000..10000, verifiez le rapport.'; return; }
    $('pitRpm').value=rpm;
    o.textContent=kmh+' km/h = '+rpm+' tr/min dans CE rapport (valable seulement dans ce rapport, embrayage non patine). Cliquez "Flasher NVRAM".';
  });
  loadState();

  $('btnLogin').addEventListener('click',function(){
    fetch('/api/login',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},
      body:form({pin:$('pin').value})})
     .then(function(r){ if(!r.ok) throw new Error(r.status); return r.json(); })
     .then(function(d){ setAuth(d.token); $('pin').value=''; })
     .catch(function(){ setAuth(''); $('authState').textContent='PIN refuse ou verrouille'; });
  });

  $('btnSave').addEventListener('click',function(){
    if(!TOKEN){ $('authState').textContent='Connectez-vous d’abord'; return; }
    var data={};
    ['shiftCutMs','minShiftRpm','pitRpm','pitKmh','wheelCircumMm','pulsesPerRev'].forEach(function(k){data[k]=$(k).value;});
    if(!$('launchRpm').disabled) data.launchRpm=$('launchRpm').value;
    if(!$('maxSlipPercent').disabled) data.maxSlipPercent=$('maxSlipPercent').value;
    fetch('/api/config',{method:'POST',
      headers:Object.assign({'Content-Type':'application/x-www-form-urlencoded'},authHeader()),
      body:form(data)})
     .then(function(r){ return r.text().then(function(t){
        if(r.status===401){ setAuth(''); }
        alert(r.ok?'Parametres flashes.':'Erreur ('+r.status+') : '+t);
     });});
  });

  $('btnKey').addEventListener('click',function(){
    fetch('/api/license',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},
      body:form({key:$('key').value})})
     .then(function(r){ return r.text().then(function(t){
        alert(r.ok?'Licence activee. Redemarrage...':'Refuse ('+r.status+') : '+t);
     });});
  });

  $('btnWifi').addEventListener('click',function(){
    if(!TOKEN){ $('wifiState').textContent='Connectez-vous d’abord'; return; }
    if(!confirm('Generer un nouveau mot de passe AP ? La carte redemarre, vous serez deconnecte.')) return;
    fetch('/api/wifi',{method:'POST',headers:authHeader()})
     .then(function(r){ if(!r.ok) throw new Error(r.status); return r.json(); })
     .then(function(d){ $('wifiState').textContent='Nouveau mot de passe : '+d.password+' (notez-le) - redemarrage...'; })
     .catch(function(){ $('wifiState').textContent='Echec (auth ?)'; });
  });

  $('btnOta').addEventListener('click',function(){
    if(!TOKEN){ $('otaState').textContent='Connectez-vous d’abord'; return; }
    var f=$('fwfile').files[0];
    if(!f){ $('otaState').textContent='Choisissez un fichier .bin'; return; }
    var fd=new FormData(); fd.append('firmware',f,f.name);
    var x=new XMLHttpRequest();
    x.open('POST','/api/ota');
    x.setRequestHeader('Authorization','Bearer '+TOKEN);
    $('otaProg').hidden=false;
    x.upload.onprogress=function(e){ if(e.lengthComputable) $('otaProg').value=Math.round(e.loaded/e.total*100); };
    x.onload=function(){ $('otaState').textContent=(x.status===200?'OK - redemarrage...':'Echec ('+x.status+') : '+x.responseText); };
    x.onerror=function(){ $('otaState').textContent='Erreur reseau'; };
    x.send(fd);
  });

  if(window.EventSource){
    var es=new EventSource('/events');
    es.addEventListener('telemetry',function(e){
      try{
        var d=JSON.parse(e.data);
        if(d.locked){
          $('tier').textContent='Licence: '+d.tier;
          $('imuState').textContent='Télémétrie temps réel et data-log : réservés à la version PRO';
          $('waterState').textContent='';
          return;
        }
        $('rpm').textContent=d.rpm;
        $('spd').textContent=(+d.speed).toFixed(1);
        $('slip').textContent=(+d.slip).toFixed(1);
        if(d.lean!=null) $('lean').textContent=(+d.lean).toFixed(0);
        if(d.imu!=null) $('imuState').textContent='IMU : '+(d.imu?'OK':'ABSENT / MUET (slip control desactive)');
        if(d.waterOk){
          var wt=+d.waterTemp;
          $('waterTemp').textContent=wt.toFixed(1);
          $('waterTemp').className='v'+(wt>=95?' warn':'');  // garder en phase avec cfg::WATER_TEMP_WARN_C
          $('waterState').textContent='Capteur eau : OK';
        } else {
          $('waterTemp').textContent='--';
          $('waterTemp').className='v';
          $('waterState').textContent='Capteur eau : absent / non lu';
        }
        $('tier').textContent='Licence: '+d.tier;
        var m=$('mode');
        m.textContent=d.mode;
        m.className='badge m'+String(d.mode).replace(/[^A-Za-z]/g,'');
      }catch(err){}
    },false);
  }
})();
</script>
</body></html>
)HTMLPAGE";
