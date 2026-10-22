'use strict';
const $ = id => document.getElementById(id);
const names = ['Ведущий', 'Ведомый 1', 'Ведомый 2', 'Ведомый 3'];
const shapes = {line:'Линия',column:'Колонна',wedge:'Клин'};
const phases = {idle:'Ожидание полёта',starting:'Подготовка и взлёт',running:'Подготовка и взлёт',ready:'Группа готова к командам',landing:'Выполняется посадка',complete:'Полёт завершён',cancelled:'Полёт прерван, посадка выполнена',error:'Проверьте журнал полёта'};
const modes = {FOLLOWING:'Строй установлен',FORMATION_CHANGE:'Перестроение',DATA_HOLD:'Ожидание измерений',DISABLED:'Управление строем выключено',HOLD:'Удержание',COLLISION_STOP:'Остановка при сближении',CLOCK_RESET:'Сброс времени',INVALID_GEOMETRY:'Неверная геометрия',DEGRADED:'Ведомый выбыл, ведущий стоит'};
const leaderStatus = {HOLD:'Удержание',MANUAL:'Ручное',DEADMAN:'Ручное · нет команд',MISSION:'Миссия',COMPLETE:'Цель достигнута',WAYPOINT_TIMEOUT:'Не достиг точки',STATE_STALE:'Нет телеметрии',PATH_REVOKED:'Маршрут отозван',FORMATION_UNAVAILABLE:'Строй не готов',GROUP_MOTION_UNAVAILABLE:'Строй не успевает',CLOCK_RESET:'Сброс времени'};
// Кнопка пульта → оси команды: вперёд, влево, вверх, поворот против часовой.
const moves = {forward:{forward:1},back:{forward:-1},left:{left:1},right:{left:-1},up:{up:1},down:{up:-1},'turn-left':{yaw:1},'turn-right':{yaw:-1}};
let current = null, sending = false, localError = '', target = null;
let view = {cx:0, cy:0, scale:12, ready:false}, dragging = null;
let manual = null, manualInFlight = false, shapeChosen = false;
// Те же ориентиры, что и в учебной площадке Gazebo: ось X красная,
// ось Y синяя, цилиндры находятся на расстоянии 25 м от начала координат.
const groundMarks = [
  {x:25, y:0, color:'#f26114', label:'X +25'},
  {x:-25, y:0, color:'#a638cc', label:'X −25'},
  {x:0, y:25, color:'#1fa647', label:'Y +25'},
  {x:0, y:-25, color:'#f2c714', label:'Y −25'},
];
const token = document.querySelector('meta[name=panel-token]').content;
const rows = names.map(name => {
  const tr = document.createElement('tr');
  for (const text of [name,'Нет связи','—','—']) { const td=document.createElement('td');td.textContent=text;tr.append(td); }
  $('drones').append(tr); return tr;
});
function setNotice(message, kind='') { $('notice').textContent=message;$('notice').className='notice '+kind; }
function setTag(id, text, kind='') { $(id).textContent=text;$(id).className='tag'+(kind?' '+kind:''); }
function post(action, values) {
  return fetch('/api/command',{method:'POST',headers:{'Content-Type':'application/json','X-Panel-Token':token},body:JSON.stringify({action,...values}),signal:AbortSignal.timeout(5000)});
}
async function command(action, values={}) {
  if (sending) return;
  sending=true;localError='';if(current) render(current);
  try {
    const response=await post(action,values);
    const result=await response.json();
    if(!response.ok) throw new Error(result.error || 'Команда не выполнена');
  } catch(error) {localError=error.message;}
  finally {sending=false;}
}
// Пульт: пока кнопка нажата, команда повторяется каждые 0,1 с; сервер без
// повторов сам переходит к торможению, поэтому потеря страницы безопасна.
async function sendManual(axes) {
  if (manualInFlight) return;
  manualInFlight=true;
  try {
    const response=await post('manual',{...axes,speed:Number(document.querySelector('input[name=speed]:checked').value)});
    if(!response.ok){const result=await response.json();localError=result.error||'Команда не выполнена';stopManual(false);}
    else if(localError&&manual) localError='';
  } catch(error) {localError=error.message;stopManual(false);}
  finally {manualInFlight=false;}
}
function startManual(button) {
  stopManual(false);
  const axes=moves[button.dataset.move];
  manual={button,axes,timer:setInterval(()=>sendManual(axes),100)};
  button.classList.add('held');sendManual(axes);
}
function stopManual(sendStop=true) {
  if(!manual) return;
  clearInterval(manual.timer);manual.button.classList.remove('held');manual=null;
  if(sendStop) post('manual',{speed:Number(document.querySelector('input[name=speed]:checked').value)}).catch(()=>{});
}
document.querySelectorAll('[data-move]').forEach(button=>{
  button.addEventListener('pointerdown',event=>{if(button.disabled)return;event.preventDefault();button.setPointerCapture(event.pointerId);startManual(button);});
  for(const name of ['pointerup','pointercancel','lostpointercapture']) button.addEventListener(name,()=>{if(manual?.button===button)stopManual();});
  button.addEventListener('contextmenu',event=>event.preventDefault());
});
window.addEventListener('blur',()=>stopManual());
document.addEventListener('visibilitychange',()=>{if(document.hidden)stopManual();});
$('manual-stop').onclick=()=>{stopManual(false);command('hold');};
$('prepare').onclick=()=>command('prepare',{record:$('record').checked});
$('resume').onclick=()=>command('resume');
$('hold').onclick=()=>{stopManual(false);command('hold');};
$('land').onclick=()=>{stopManual(false);command('land');};
$('go-to-target').onclick=()=>target&&command('goal',{x:target.x,y:target.y});
$('clear-target').onclick=()=>{target=null;$('target-label').textContent='Щёлкните по карте, чтобы выбрать точку';$('clear-target').disabled=true;$('go-to-target').disabled=true;draw();};
document.querySelectorAll('input[name=formation]').forEach(input=>input.addEventListener('change',()=>{shapeChosen=true;}));
$('formation-form').onsubmit=event=>{event.preventDefault();if(!$('apply').disabled) command('formation',{formation:document.querySelector('input[name=formation]:checked').value,scale:Number($('scale').value),duration:Number($('duration').value)});};
$('altitude-form').onsubmit=event=>{event.preventDefault();if(!$('set-altitude').disabled) command('altitude',{z:Number($('altitude').value)});};
function render(state) {
  current=state;
  const flight=state.flight,active=['starting','running','ready','landing'].includes(flight.phase);
  const available=state.connected&&!active&&!sending&&state.action.state!=='pending'&&state.drones.every(d=>!d.armed);
  const operator=state.connected&&flight.phase==='ready'&&flight.kind==='operator';
  const ready=operator&&!sending&&state.action.state!=='pending';
  const leader=state.drones[0];
  const flying=operator&&leader.fresh&&leader.armed&&leader.mode==='14'&&leader.position[2]>=1;
  $('connection').querySelector('b').textContent=state.connected?'Четыре аппарата на связи':'Ожидание свежих данных';
  $('connection').className='pill'+(state.connected?' connected':'');
  // Аппарат вне Offboard при готовой сессии — эскалация моста; возврат по решению оператора.
  const parked=operator&&state.drones.some(d=>d.fresh&&d.armed&&d.mode!=='14'&&d.health!=='FAILSAFE_LAND');
  $('resume').hidden=!parked;$('resume').disabled=!parked||!ready;
  $('prepare').disabled=!available;$('hold').disabled=!operator||sending;
  $('land').disabled=!(active||state.drones.some(d=>d.armed))||flight.phase==='landing'||sending;
  $('apply').disabled=!ready||!state.formation_settled;
  $('set-altitude').disabled=!ready||!flying;
  $('go-to-target').disabled=!target||!ready||sending;
  $('clear-target').disabled=!target;
  document.querySelectorAll('[data-move]').forEach(button=>button.disabled=!flying);
  if(!flying) stopManual(false);
  $('manual-stop').disabled=!operator||sending;
  $('record').disabled=active;
  $('flight-status').textContent=phases[flight.phase]||'Ожидание';
  $('flight-dot').className='dot'+(flight.phase==='ready'?' ready':flight.phase==='error'?' error':active?' busy':'');
  const swarm=state.swarm_mode;
  setTag('swarm-mode',swarm?swarm.mode:'—',!swarm?'':swarm.mode==='FAILSAFE'?'warn':['DEGRADED','TAKEOFF','FORMING','LAND','FORMATION_CHANGE'].includes(swarm.mode)?'busy':'ok');
  $('swarm-mode').title=swarm?`Режим группы (ТЗ §8): ${swarm.reason}`:'Режим группы по ТЗ §8';
  $('sim-time').textContent=`Время ${state.time.toFixed(1)} с`;
  if(localError) setNotice(localError,'error');
  else if(flight.phase==='error') setNotice('Полёт завершился с ошибкой. Проверьте события и подтверждение посадки.','error');
  else if(flight.phase==='landing') setNotice('Выполняется посадка. Дождитесь разоружения всех аппаратов.','pending');
  else if(['starting','running'].includes(flight.phase)) setNotice('Выполняется подготовка: взлёт и включение строя. Посадка доступна в любой момент.','pending');
  else if(['complete','cancelled'].includes(flight.phase)) setNotice('Посадка подтверждена. Можно начать новый полёт.');
  else if(flight.phase==='ready'&&state.action.state==='ok'&&state.action.message.startsWith('Подготовка')) setNotice('Группа готова: выберите строй, ведите ведущего вручную или укажите точку на карте.');
  else setNotice(state.action.message,state.action.state==='error'?'error':state.action.state==='pending'?'pending':'');
  state.drones.forEach((drone,i)=>{
    const cells=rows[i].children;
    cells[1].textContent=!drone.fresh?'Нет свежих данных':!drone.armed?'На земле':drone.health==='FAILSAFE_LAND'?'Посадка: отказ данных':drone.health==='FAILSAFE_HOLD'?'Удержание PX4: отказ данных':drone.mode==='18'?'Посадка':drone.position[2]<.7?'Подготовка':'В полёте';
    cells[1].className=drone.fresh?'fresh':'stale';
    cells[2].textContent=drone.fresh?`${drone.position[2].toFixed(2)} м`:'—';
    cells[3].textContent=drone.fresh?`${Math.hypot(...drone.velocity).toFixed(2)} м/с`:'—';
  });
  const formation=state.formation;
  const settling=formation?.mode==='FOLLOWING'&&!state.formation_settled;
  // До выбора оператора форма показывает текущий строй.
  if(formation&&!shapeChosen&&$('formation-'+formation.formation)){$('formation-'+formation.formation).checked=true;$('scale').value=Number(formation.scale);}
  if(!formation) setTag('formation-status','Нет данных');
  else setTag('formation-status',`${shapes[formation.formation]||''} · ×${Number(formation.scale).toFixed(2)} · ${settling?'Занимают места':modes[formation.mode]||'Ожидание'}`,
    formation.mode==='FOLLOWING'&&!settling?'ok':['FORMATION_CHANGE'].includes(formation.mode)||settling?'busy':'warn');
  const errors=formation&&Array.isArray(formation.errors)&&formation.errors.length===3?formation.errors:null;
  $('errors').replaceChildren(...(errors?errors.map((e,i)=>{const chip=document.createElement('span');chip.className='chip '+(e===null?'bad':e<.3?'good':e<.6?'mid':'bad');chip.textContent=`${names[i+1]}: ${e===null?'выбыл':Number(e).toFixed(2)+' м'}`;return chip;}):[Object.assign(document.createElement('span'),{className:'chip',textContent:'—'})]));
  $('altitude-now').textContent=leader.fresh?leader.position[2].toFixed(1):'—';
  const execution=state.execution;
  const status=execution?.status;
  setTag('leader-mode',execution?(state.manual?'Движение':leaderStatus[status]||status):'—',
    state.manual||status==='MANUAL'?'ok':status==='DEADMAN'?'busy':['STATE_STALE','FORMATION_UNAVAILABLE','GROUP_MOTION_UNAVAILABLE'].includes(status)?'warn':'');
  setTag('mission-status',status==='MISSION'?`Миссия · точка ${execution.waypoint+1}/${execution.total}`:status==='COMPLETE'?'Цель достигнута':target?'Точка выбрана':'Точка не выбрана',
    status==='MISSION'?'busy':status==='COMPLETE'?'ok':'');
  // Курс по компасу: 0° — север (+Y), 90° — восток (+X).
  $('leader-heading').textContent=leader.fresh?`${Math.round(((90-leader.yaw*180/Math.PI)%360+360)%360)}°`:'—';
  $('leader-speed').textContent=leader.fresh?`${Math.hypot(...leader.velocity).toFixed(1)} м/с`:'—';
  $('leader-altitude').textContent=leader.fresh?`${leader.position[2].toFixed(1)} м`:'—';
  const translate=line=>names.reduce((text,name,i)=>text.replaceAll(`uav_${i}`,name),line).replaceAll('column','колонна').replaceAll('wedge','клин').replaceAll('line','линия');
  const text=flight.logs.slice(-24).map(translate).join('\n')||'Событий пока нет.';
  if($('events').textContent!==text){$('events').textContent=text;$('events').scrollTop=$('events').scrollHeight;}
  $('output').textContent=flight.output?`Запись: ${flight.output.split('/').slice(-3).join('/')}`:'';
  draw();
}
function draw() {
  const canvas=$('map'),rect=canvas.getBoundingClientRect(),ratio=window.devicePixelRatio||1;
  canvas.width=Math.round(rect.width*ratio);canvas.height=Math.round(rect.height*ratio);
  const ctx=canvas.getContext('2d');ctx.scale(ratio,ratio);
  const w=rect.width,h=rect.height,valid=(current?.drones||[]).filter(d=>d.fresh);
  if(!valid.length){ctx.fillStyle='#8fa1b2';ctx.font='13px system-ui';ctx.textAlign='center';ctx.fillText('Положение появится после подключения',w/2,h/2);return;}
  if(!view.ready){const xs=valid.map(d=>d.position[0]).concat(groundMarks.map(mark=>mark.x)),ys=valid.map(d=>d.position[1]).concat(groundMarks.map(mark=>mark.y));view.cx=(Math.min(...xs)+Math.max(...xs))/2;view.cy=(Math.min(...ys)+Math.max(...ys))/2;view.scale=Math.min((w-90)/Math.max(16,Math.max(...xs)-Math.min(...xs)),(h-80)/Math.max(16,Math.max(...ys)-Math.min(...ys)));view.ready=true;}
  const cx=view.cx,cy=view.cy,scale=view.scale;
  const px=x=>w/2+(x-cx)*scale,py=y=>h/2-(y-cy)*scale;
  ctx.lineWidth=1;ctx.strokeStyle='#1a2632';ctx.fillStyle='#6f8293';ctx.font='10px system-ui';ctx.textAlign='center';
  const step=scale<8?10:2;
  for(let x=Math.floor((cx-w/scale/2)/step)*step;x<cx+w/scale/2;x+=step){ctx.beginPath();ctx.moveTo(px(x),0);ctx.lineTo(px(x),h);ctx.stroke();ctx.fillText(x.toFixed(0),px(x),h-9);}
  for(let y=Math.floor((cy-h/scale/2)/step)*step;y<cy+h/scale/2;y+=step){ctx.beginPath();ctx.moveTo(0,py(y));ctx.lineTo(w,py(y));ctx.stroke();}
  // Цвета и координаты совпадают с разметкой training_ground в Gazebo.
  ctx.lineWidth=2;
  ctx.strokeStyle='#ef604f';ctx.beginPath();ctx.moveTo(px(-100),py(0));ctx.lineTo(px(100),py(0));ctx.stroke();
  ctx.strokeStyle='#4f86ed';ctx.beginPath();ctx.moveTo(px(0),py(-100));ctx.lineTo(px(0),py(100));ctx.stroke();
  ctx.font='10px system-ui';ctx.textAlign='left';ctx.fillStyle='#ef8a7d';ctx.fillText('X',Math.min(w-14,px(100)-8),Math.max(12,py(0)-6));
  ctx.fillStyle='#82aaf4';ctx.fillText('Y',Math.min(w-14,px(0)+6),Math.max(12,py(100)+12));
  groundMarks.forEach(mark=>{const x=px(mark.x),y=py(mark.y);ctx.fillStyle=mark.color;ctx.strokeStyle='#0b131b';ctx.lineWidth=1.5;ctx.beginPath();ctx.arc(x,y,5.5,0,Math.PI*2);ctx.fill();ctx.stroke();ctx.font='10px system-ui';ctx.textAlign='center';ctx.fillText(mark.label,x,y-9);});
  const occupied=valid.map(drone=>({x:px(drone.position[0])-8,y:py(drone.position[1])-8,w:16,h:16}));
  valid.forEach(drone=>{
    ctx.lineWidth=1.5;
    const x=px(drone.position[0]),y=py(drone.position[1]);ctx.fillStyle=drone.id===0?'#ff8a73':'#c9dbe6';
    if(drone.id===0){ctx.save();ctx.fillStyle='rgba(255,138,115,.16)';ctx.beginPath();ctx.arc(x,y,13,0,Math.PI*2);ctx.fill();ctx.restore();}
    ctx.beginPath();ctx.arc(x,y,drone.id===0?6:4.5,0,Math.PI*2);ctx.fill();
    ctx.strokeStyle=ctx.fillStyle;ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(x+14*Math.cos(drone.yaw),y-14*Math.sin(drone.yaw));ctx.stroke();
    ctx.font='11px system-ui';ctx.textAlign='left';
    const label=names[drone.id],width=ctx.measureText(label).width;
    let chosen=null;
    for(let ring=0;ring<5&&!chosen;ring++){
      const distance=12+ring*16;
      for(const [lx,ly] of [[x+distance,y-distance],[x+distance,y+distance+8],[x-width-distance,y-distance],[x-width-distance,y+distance+8]]){
        const candidate={x:lx-3,y:ly-12,w:width+6,h:17};
        if(candidate.x<2||candidate.y<2||candidate.x+candidate.w>w-2||candidate.y+candidate.h>h-2)continue;
        if(!occupied.some(other=>candidate.x<other.x+other.w&&candidate.x+candidate.w>other.x&&candidate.y<other.y+other.h&&candidate.y+candidate.h>other.y)){chosen={...candidate,lx,ly};break;}
      }
    }
    if(chosen){
      occupied.push(chosen);ctx.strokeStyle=ctx.fillStyle;ctx.lineWidth=.7;
      ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(Math.max(chosen.x,Math.min(x,chosen.x+chosen.w)),Math.max(chosen.y,Math.min(y,chosen.y+chosen.h)));ctx.stroke();
      ctx.strokeStyle='#0b131b';ctx.lineWidth=3;ctx.strokeText(label,chosen.lx,chosen.ly);ctx.fillText(label,chosen.lx,chosen.ly);
    }
  });
  if(target){const x=px(target.x),y=py(target.y);ctx.strokeStyle='#f6c56b';ctx.lineWidth=2;ctx.beginPath();ctx.arc(x,y,8,0,Math.PI*2);ctx.stroke();ctx.beginPath();ctx.moveTo(x-12,y);ctx.lineTo(x+12,y);ctx.moveTo(x,y-12);ctx.lineTo(x,y+12);ctx.stroke();}
}
const map=$('map');
map.addEventListener('wheel',event=>{event.preventDefault();const rect=map.getBoundingClientRect(),mx=event.clientX-rect.left,my=event.clientY-rect.top;const before=[(mx-rect.width/2)/view.scale+view.cx,(rect.height/2-my)/view.scale+view.cy];view.scale=Math.max(3,Math.min(80,view.scale*Math.exp(-event.deltaY*.001)));view.cx=before[0]-(mx-rect.width/2)/view.scale;view.cy=before[1]+(my-rect.height/2)/view.scale;view.ready=true;draw();},{passive:false});
map.addEventListener('pointerdown',event=>{map.setPointerCapture(event.pointerId);dragging={x:event.clientX,y:event.clientY,cx:view.cx,cy:view.cy};});
map.addEventListener('pointercancel',()=>{dragging=null;});
map.addEventListener('lostpointercapture',()=>{dragging=null;});
map.addEventListener('pointermove',event=>{if(!dragging)return;view.cx=dragging.cx-(event.clientX-dragging.x)/view.scale;view.cy=dragging.cy+(event.clientY-dragging.y)/view.scale;draw();});
map.addEventListener('pointerup',event=>{if(!dragging)return;const moved=Math.hypot(event.clientX-dragging.x,event.clientY-dragging.y);if(moved<5){const rect=map.getBoundingClientRect();target={x:view.cx+(event.clientX-rect.left-rect.width/2)/view.scale,y:view.cy-(event.clientY-rect.top-rect.height/2)/view.scale};$('target-label').textContent=`Цель: X=${target.x.toFixed(1)} м, Y=${target.y.toFixed(1)} м`;}$('clear-target').disabled=!target;dragging=null;draw();});
new ResizeObserver(draw).observe($('map'));
async function poll(){
  try{const response=await fetch('/api/state',{signal:AbortSignal.timeout(3000)});if(!response.ok)throw new Error();render(await response.json());}
  catch{stopManual(false);setNotice('Панель недоступна. Проверьте её терминал; закрытие страницы не завершает полёт.','error');$('connection').querySelector('b').textContent='Нет связи с панелью';$('connection').className='pill';document.querySelectorAll('button').forEach(b=>b.disabled=true);current=null;draw();}
  setTimeout(poll,500);
}
poll();
