// control.js — Web UI MQTT client for controlling the hand
(function(){
  const broker = 'broker.hivemq.com';
  const useSecure = true; // true -> wss on 8884, false -> ws on 8000
  const port = useSecure ? 8884 : 8000;
  const path = '/mqtt';

  const topicMode = 'hand/mode';
  const topicManual = 'hand/manual';
  const topicStatus = 'hand/status';

  const clientId = 'webctrl-' + Math.random().toString(16).substr(2,8);
  let client = null; // created after Paho is available

  // UI elements
  const btnEmg = document.getElementById('btnEmg');
  const btnManual = document.getElementById('btnManual');
  const mqttStatus = document.getElementById('mqttStatus');
  const deviceMode = document.getElementById('deviceMode');

  const sThumb = document.getElementById('sThumb'); const vThumb = document.getElementById('vThumb');
  const sIndex = document.getElementById('sIndex'); const vIndex = document.getElementById('vIndex');
  const sMiddle = document.getElementById('sMiddle'); const vMiddle = document.getElementById('vMiddle');
  const sRing = document.getElementById('sRing'); const vRing = document.getElementById('vRing');
  const sPinky = document.getElementById('sPinky'); const vPinky = document.getElementById('vPinky');

  const btnSend = document.getElementById('btnSend');
  const autoSend = document.getElementById('autoSend');
  const lastPayload = document.getElementById('lastPayload');

  function setConnState(connected){
    mqttStatus.textContent = connected ? 'Kết nối ✅' : 'Đang ngắt ❌';
    mqttStatus.className = connected ? 'status' : 'status muted';
    // also update inline status in status tab if present
    const mqttInline = document.getElementById('mqttStatusInline');
    if(mqttInline) mqttInline.textContent = connected ? 'Kết nối ✅' : 'Đang ngắt ❌';
    // Disable mode buttons until we have an MQTT connection
    btnEmg.disabled = !connected;
    btnManual.disabled = !connected;
  }

  function setDeviceMode(mode){
    deviceMode.textContent = mode;
    deviceMode.className = 'status';
    const deviceInline = document.getElementById('deviceModeInline');
    if(deviceInline) deviceInline.textContent = mode;
    // enable manual controls only if device is in MANUAL
    const enabled = (mode && mode.toLowerCase() === 'manual');
    [sThumb, sIndex, sMiddle, sRing, sPinky, btnSend, autoSend].forEach(el => el.disabled = !enabled);
    if(enabled) {
      btnManual.classList.add('active');
      btnEmg.classList.remove('active');
    } else {
      btnEmg.classList.add('active');
      btnManual.classList.remove('active');
    }
  }

  // Update displayed slider values
  function bindSlider(sl, valEl){
    sl.addEventListener('input', ()=>{
      valEl.textContent = sl.value;
      if(autoSend.checked && !sl.disabled) sendManual();
    });
  }
  bindSlider(sThumb, vThumb);
  bindSlider(sIndex, vIndex);
  bindSlider(sMiddle, vMiddle);
  bindSlider(sRing, vRing);
  bindSlider(sPinky, vPinky);

  // Generic publish that supports Paho and mqtt.js
  let clientImpl = null; // 'paho' or 'mqttjs'

  function publish(topic, payload){
    if(clientImpl === 'paho'){
      if(client && client.isConnected()){
        const msg = new Paho.MQTT.Message(payload);
        msg.destinationName = topic;
        client.send(msg);
        lastPayload.textContent = payload;
      } else {
        console.warn('Paho client not connected, cannot publish', topic, payload);
      }
    } else if(clientImpl === 'mqttjs'){
      if(client && client.connected){
        client.publish(topic, payload);
        lastPayload.textContent = payload;
      } else {
        console.warn('mqtt.js client not connected, cannot publish', topic, payload);
      }
    } else {
      console.warn('No MQTT client available to publish', topic, payload);
    }
  }

  function publishMode(mode){
    publish(topicMode, mode);
  }

  function sendManual(){
    const payload = [sThumb.value, sIndex.value, sMiddle.value, sRing.value, sPinky.value].join(':');
    publish(topicManual, payload);
  }

  btnSend.addEventListener('click', sendManual);
  btnEmg.addEventListener('click', ()=>{ publishMode('emg'); });
  btnManual.addEventListener('click', ()=>{ publishMode('manual'); });

  // Create the client and setup handlers (called after loader is done)
  function initClientPaho(){
    clientImpl = 'paho';
    client = new Paho.MQTT.Client(broker, Number(port), path, clientId);

    client.onConnectionLost = function(responseObject){
      console.log('Paho: Connection lost', responseObject);
      setConnState(false);
      setDeviceMode('?');
      setTimeout(connect, 2000);
    };

    client.onMessageArrived = function(message){
      console.log('Paho: Msg arrived:', message.destinationName, message.payloadString);
      if(message.destinationName === topicStatus){
        try{ const obj = JSON.parse(message.payloadString); if(obj.mode) setDeviceMode(obj.mode);} catch(e){ console.warn('Invalid status payload', e); }
      }
    };

    setDeviceMode('?');
    setConnState(false);
    connect();
  }

  function initClientMqttJs(){
    clientImpl = 'mqttjs';
    const protocol = useSecure ? 'wss' : 'ws';
    const url = protocol + '://' + broker + ':' + port + (path ? path : '');
    console.log('Connecting mqtt.js to', url);
    client = mqtt.connect(url, { clientId: clientId, rejectUnauthorized: false, reconnectPeriod: 2000 });

    client.on('connect', function(){
      console.log('mqtt.js: Connected');
      setConnState(true);
      client.subscribe(topicStatus);
    });

    client.on('reconnect', function(){ console.log('mqtt.js: reconnecting'); setConnState(false); });
    client.on('close', function(){ console.log('mqtt.js: closed'); setConnState(false); setDeviceMode('?'); });
    client.on('error', function(err){ console.warn('mqtt.js error', err); });

    client.on('message', function(topic, payload){
      const msg = payload.toString();
      console.log('mqtt.js: Msg arrived:', topic, msg);
      if(topic === topicStatus){ try{ const obj = JSON.parse(msg); if(obj.mode) setDeviceMode(obj.mode); } catch(e){ console.warn('Invalid status payload', e); } }
    });
  }

  function initClient(){
    // prefer Paho if available
    if(window.Paho && window.Paho.MQTT){ initClientPaho(); return; }
    if(window.mqtt){ initClientMqttJs(); return; }
    console.error('No MQTT client library available during init');
  }

  function connect(){
    if(clientImpl === 'paho'){
      if(!client) return; setConnState(false);
      client.connect({ onSuccess: onConnect, onFailure: onFail, useSSL: useSecure });
    } else if(clientImpl === 'mqttjs'){
      // mqtt.js auto-connects on creation; nothing to do here
    }
  }

  function onConnect(){
    console.log('Connected to MQTT');
    setConnState(true);
    if(clientImpl === 'paho') client.subscribe(topicStatus);
  }

  function onFail(err){
    console.warn('MQTT connect failed', err);
    setConnState(false);
    setTimeout(connect, 2000);
  }

  // Fallback loader for MQTT libs (tries multiple CDNs sequentially)
  async function ensurePaho(callback){
    if(window.Paho && window.Paho.MQTT) return callback();

    const pahoUrls = [
      'https://cdnjs.cloudflare.com/ajax/libs/paho-mqtt/1.1.0/mqttws31.min.js',
      'https://unpkg.com/paho-mqtt@1.1.0/paho-mqtt-min.js',
      'https://cdn.jsdelivr.net/npm/paho-mqtt@1.1.0/paho-mqtt-min.js'
    ];

    const mqttJsUrls = [
      'https://unpkg.com/mqtt/dist/mqtt.min.js',
      'https://cdn.jsdelivr.net/npm/mqtt/dist/mqtt.min.js'
    ];

    for(const url of pahoUrls){
      try{
        await new Promise((resolve, reject) => {
          const s = document.createElement('script'); s.src = url;
          s.onload = () => setTimeout(() => (window.Paho && window.Paho.MQTT) ? resolve() : reject(new Error('Paho did not initialize')), 50);
          s.onerror = () => reject(new Error('Load error'));
          document.head.appendChild(s);
        });
        console.log('Loaded Paho from', url);
        return callback();
      } catch(e){ console.warn('Failed to load Paho from', url, e.message); }
    }

    // try mqtt.js as fallback
    for(const url of mqttJsUrls){
      try{
        await new Promise((resolve, reject) => {
          const s = document.createElement('script'); s.src = url;
          s.onload = () => setTimeout(() => (window.mqtt) ? resolve() : reject(new Error('mqtt did not initialize')), 50);
          s.onerror = () => reject(new Error('Load error'));
          document.head.appendChild(s);
        });
        console.log('Loaded mqtt.js from', url);
        return callback();
      } catch(e){ console.warn('Failed to load mqtt.js from', url, e.message); }
    }

    console.error('Failed to load any MQTT library from CDNs after all attempts');
  }

  // Ensure Paho/mqtt.js then init
  ensurePaho(initClient);

  // --- Tabs logic (switch local sections and navigation) ---
  function initTabs(){
    const tabBtns = document.querySelectorAll('.tab-btn');
    const tabContents = document.querySelectorAll('.tab-content');
    function showTab(name){
      if(!name) name = 'manual';
      tabBtns.forEach(b => b.classList.toggle('active', b.dataset.tab === name));
      tabContents.forEach(c => c.style.display = (c.id === 'tab-' + name) ? '' : 'none');
      try{ history.replaceState(null, '', '#'+name); } catch(e){}
    }
    tabBtns.forEach(b => b.addEventListener('click', ()=> showTab(b.dataset.tab)));
    const sel = document.getElementById('tab-pages-select');
    if(sel) sel.addEventListener('change', ()=> { if(sel.value) location.href = sel.value; });

    // initialize based on hash or default
    const hash = (location.hash || '').replace('#','');
    showTab(hash || 'manual');
  }
  window.addEventListener('load', initTabs);
})();