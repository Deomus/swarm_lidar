'use strict';
const $ = id => document.getElementById(id);
const names = ['Ведущий', 'Ведомый 1', 'Ведомый 2', 'Ведомый 3'];
const shapes = {line:'Линия',column:'Колонна',wedge:'Клин'};
const phases = {idle:'Ожидание полёта',starting:'Подготовка и взлёт',running:'Выполняется сценарий',ready:'Группа готова к командам',landing:'Выполняется посадка',complete:'Полёт завершён',cancelled:'Полёт прерван, посадка выполнена',error:'Проверьте журнал полёта'};
const modes = {FOLLOWING:'Строй установлен',FORMATION_CHANGE:'Перестроение',DATA_HOLD:'Ожидание измерений',DISABLED:'Управление строем выключено',HOLD:'Удержание',COLLISION_STOP:'Остановка при сближении',CLOCK_RESET:'Сброс времени'};
let current = null, sending = false, localError = '';
const token = document.querySelector('meta[name=panel-token]').content;
const rows = names.map((name, i) => {
  const tr = document.createElement('tr');
  for (const text of [name,'Нет связи','—','—']) { const td=document.createElement('td');td.textContent=text;tr.append(td); }
  $('drones').append(tr); return tr;
});
function setNotice(message, kind='') { $('notice').textContent=message;$('notice').className='notice '+kind; }
async function command(action, values={}) {
  if (sending) return;
  sending=true;localError='';if(current) render(current);
  try {
    const response=await fetch('/api/command',{method:'POST',headers:{'Content-Type':'application/json','X-Panel-Token':token},body:JSON.stringify({action,...values}),signal:AbortSignal.timeout(5000)});
    const result=await response.json();
    if(!response.ok) throw new Error(result.error || 'Команда не выполнена');
  } catch(error) {localError=error.message;}
  finally {sending=false;}
}
$('prepare').onclick=()=>command('prepare',{record:$('record').checked});
$('hold').onclick=()=>command('hold');
$('land').onclick=()=>command('land');
document.querySelectorAll('[data-scenario]').forEach(button=>button.onclick=()=>command(button.dataset.scenario,{record:$('record').checked}));
$('formation-form').onsubmit=event=>{event.preventDefault();if(!$('apply').disabled) command('formation',{formation:$('formation').value,scale:Number($('scale').value),duration:Number($('duration').value)});};
function render(state) {
  current=state;
  const flight=state.flight,active=['starting','running','ready','landing'].includes(flight.phase);
  const available=state.connected&&!active&&!sending&&state.action.state!=='pending'&&state.drones.every(d=>!d.armed);
  const ready=state.connected&&flight.phase==='ready'&&flight.kind==='operator'&&!sending&&state.action.state!=='pending';
  $('connection').textContent=state.connected?'Четыре аппарата подключены':'Ожидание свежих данных';
  $('connection').className='badge'+(state.connected?' connected':'');
  $('prepare').disabled=!available;$('hold').disabled=!ready;
  $('land').disabled=!(active||state.drones.some(d=>d.armed))||flight.phase==='landing'||sending;
  $('apply').disabled=!ready||!state.formation||state.formation.mode!=='FOLLOWING';
  document.querySelectorAll('[data-scenario]').forEach(button=>button.disabled=!available);
  $('record').disabled=active;
  $('flight-status').textContent=phases[flight.phase]||'Ожидание';
  $('sim-time').textContent=`Время ${state.time.toFixed(1)} с`;
  if(localError) setNotice(localError,'error');
  else if(flight.phase==='error') setNotice('Полёт завершился с ошибкой. Проверьте события и подтверждение посадки.','error');
  else if(flight.phase==='landing') setNotice('Выполняется посадка. Дождитесь разоружения всех аппаратов.','pending');
  else if(['starting','running'].includes(flight.phase)) setNotice('Выполняется подготовка или сценарий. Посадка доступна в любой момент.','pending');
  else if(['complete','cancelled'].includes(flight.phase)) setNotice('Посадка подтверждена. Можно начать новый полёт.');
  else if(flight.phase==='ready'&&state.action.state==='ok'&&state.action.message.startsWith('Подготовка')) setNotice('Группа готова. Выберите строй или завершите полёт.');
  else setNotice(state.action.message,state.action.state==='error'?'error':state.action.state==='pending'?'pending':'');
  state.drones.forEach((drone,i)=>{
    const cells=rows[i].children;
    cells[1].textContent=!drone.fresh?'Нет свежих данных':!drone.armed?'На земле':drone.mode==='18'?'Посадка':drone.position[2]<.7?'Подготовка':'В полёте';
    cells[1].className=drone.fresh?'fresh':'stale';
    cells[2].textContent=drone.fresh?`${drone.position[2].toFixed(2)} м`:'—';
    cells[3].textContent=drone.fresh?`${Math.hypot(...drone.velocity).toFixed(2)} м/с`:'—';
  });
  const formation=state.formation;
  $('formation-status').textContent=formation?`${shapes[formation.formation]||''} · ${modes[formation.mode]||'Ожидание'}`:'Нет свежих данных';
  $('errors').textContent=formation&&Array.isArray(formation.errors)&&formation.errors.length===3?`Ошибки по LiDAR: ${formation.errors.map(e=>Number(e).toFixed(2)).join(' / ')} м`:'Ошибки строя: —';
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
  if(!valid.length){ctx.fillStyle='#9babb9';ctx.font='13px system-ui';ctx.textAlign='center';ctx.fillText('Положение появится после подключения',w/2,h/2);return;}
  const xs=valid.map(d=>d.position[0]),ys=valid.map(d=>d.position[1]);
  const cx=(Math.min(...xs)+Math.max(...xs))/2,cy=(Math.min(...ys)+Math.max(...ys))/2;
  const scale=Math.min((w-90)/Math.max(16,Math.max(...xs)-Math.min(...xs)),(h-80)/Math.max(16,Math.max(...ys)-Math.min(...ys)));
  const px=x=>w/2+(x-cx)*scale,py=y=>h/2-(y-cy)*scale;
  ctx.lineWidth=1;ctx.strokeStyle='#27343f';ctx.fillStyle='#8295a6';ctx.font='10px system-ui';ctx.textAlign='center';
  const step=scale<8?10:2;
  for(let x=Math.floor((cx-w/scale/2)/step)*step;x<cx+w/scale/2;x+=step){ctx.beginPath();ctx.moveTo(px(x),0);ctx.lineTo(px(x),h);ctx.stroke();ctx.fillText(x.toFixed(0),px(x),h-9);}
  for(let y=Math.floor((cy-h/scale/2)/step)*step;y<cy+h/scale/2;y+=step){ctx.beginPath();ctx.moveTo(0,py(y));ctx.lineTo(w,py(y));ctx.stroke();}
  valid.forEach(drone=>{
    const x=px(drone.position[0]),y=py(drone.position[1]);ctx.fillStyle=drone.id===0?'#f09588':'#cde8dc';ctx.beginPath();ctx.arc(x,y,drone.id===0?6:4.5,0,Math.PI*2);ctx.fill();
    ctx.strokeStyle=ctx.fillStyle;ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(x+14*Math.cos(drone.yaw),y-14*Math.sin(drone.yaw));ctx.stroke();
    ctx.font='11px system-ui';ctx.textAlign=drone.id===0?'left':'center';
    ctx.fillText(names[drone.id],drone.id===0?x+11:x,drone.id===0?y+4:y-13);
  });
}
new ResizeObserver(draw).observe($('map'));
async function poll(){
  try{const response=await fetch('/api/state',{signal:AbortSignal.timeout(3000)});if(!response.ok)throw new Error();render(await response.json());}
  catch{setNotice('Панель недоступна. Проверьте её терминал; закрытие страницы не завершает полёт.','error');$('connection').textContent='Нет связи с панелью';$('connection').className='badge';document.querySelectorAll('button').forEach(b=>b.disabled=true);current=null;draw();}
  setTimeout(poll,500);
}
poll();
