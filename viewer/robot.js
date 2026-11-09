'use strict';
// Robot status (#view/ID/robot), drawn here from testground/robot/status_server.py's /status:
// battery charge, HARP's harness state and each motor's casing and winding temperature
// against the limits the harness stops the robot at. Read-only. Loaded before app.js, whose
// helpers ($, node) it uses only when called.
const LIMBS=[['Left leg',0,6],['Right leg',6,12],['Waist',12,15],['Left arm',15,22],['Right arm',22,29]];
const NEAR=0.8;   // a reading this share of its limit or more is near it
const short=name=>name.replace(/^(left|right)_/,'').replace(/_/g,' ');
const degrees=v=>v==null?'–':`${Math.round(v)} °C`;
function heat(motor,limits) {
  // How close a motor is to the harness's stop: the larger share of either limit.
  const parts=[motor.casing_c!=null?motor.casing_c/limits.casing_c:0,motor.winding_c!=null?motor.winding_c/limits.winding_c:0];
  return Math.max(...parts);
}
function tone(share){return share>=1?'bad':share>=NEAR?'warn':'';}
function hottest(d){
  let best=null;for(const m of d.motors||[]){const h=heat(m,d.limits);if(!best||h>best.h)best={m,h};}return best;
}
function harnessLine(h,online) {
  // What HARP's harness is doing, in words, and how it should look.
  if(!online)return ['Harness not running',''];
  if(h.abort_reason)return [`Harness stopped it: ${String(h.abort_reason).replace(/_/g,' ')}`,'bad'];
  if(h.handover_complete)return ['Our policy has control','ok'];
  if(h.armed)return [`Armed · ${String(h.startup_phase||'starting').replace(/_/g,' ')}`,'warn'];
  if(h.arm_rejection_reason)return [`Not armed: ${String(h.arm_rejection_reason).replace(/_/g,' ')}`,'warn'];
  return ['Waiting, not armed',''];
}
function batteryTone(p,limits){return p==null?'':p<=limits.battery_percent?'bad':p<25?'warn':'ok';}

// Home's tile: the same readings as chips (app.js calls this through its tiles table).
function robotTile(s) {
  const d=s.detail||{},limits=d.limits||{casing_c:85,winding_c:120,battery_percent:10},h=d.harness||{};
  const harnessOn=Boolean(s.feeds.harness&&s.feeds.harness.online),[headline]=harnessLine(h,harnessOn),chips=[];
  const p=d.battery&&d.battery.percent;
  chips.push(chip('Battery',p==null?'No reading':`${Math.round(p)}%`,p==null?'warn':batteryTone(p,limits)));
  const hot=hottest(d);
  if(hot)chips.push(chip('Hottest',`${hot.m.name.replace(/_/g,' ')} ${degrees(Math.max(hot.m.casing_c??0,hot.m.winding_c??0))}`,tone(hot.h)||'ok'));
  if(h.reported_fsm_id!=null)chips.push(chip('FSM',String(h.reported_fsm_id)));
  if(d.lowstate&&d.lowstate.hz!=null)chips.push(chip('Joint state',`${d.lowstate.hz} Hz`));
  return {headline,chips,foot:'Read-only, sends nothing'};
}

function showRobot(body,source) {
  let page=body.querySelector('.robot-page');
  if(!page){
    page=node('div','robot-page');
    page.append(node('div','robot-summary'),node('h2','section-title','Motor temperatures'),node('p','robot-note'),node('div','robot-limbs'));
    body.prepend(page);
  }
  const d=source.detail||{},limits=d.limits||{casing_c:85,winding_c:120,battery_percent:10},h=d.harness||{};
  const harnessOn=Boolean(source.feeds.harness&&source.feeds.harness.online);
  const key=JSON.stringify([d,harnessOn]);if(page.dataset.key===key)return;page.dataset.key=key;
  // Summary tiles: battery, harness, hottest motor, the joint state feed.
  const p=d.battery&&d.battery.percent,[line,lineTone]=harnessLine(h,harnessOn),hot=hottest(d);
  const tile=(label,value,sub,cls)=>{const t=node('div','robot-tile'+(cls?' '+cls:''));t.append(node('span','tile-label',label),node('span','tile-value',value),node('span','tile-sub',sub));return t;};
  const battery=tile('Battery',p==null?'No reading':`${Math.round(p)}%`,`the harness stops below ${limits.battery_percent}%`,batteryTone(p,limits));
  if(p!=null){const meter=node('span','meter');const fill=node('span','meter-fill');fill.style.width=`${Math.max(0,Math.min(100,p))}%`;meter.append(fill);battery.append(meter);}
  const harness=tile('HARP\'s harness',line,harnessOn?[h.handover_complete?'handover complete':h.armed?'handover pending':'',h.reported_fsm_id!=null&&`FSM ${h.reported_fsm_id}`,h.hardware_output_enabled===false&&'output off (dry run)'].filter(Boolean).join(' · '):(source.feeds.harness||{}).reason||'',lineTone);
  const hottestTile=hot?tile('Hottest motor',hot.m.name.replace(/_/g,' '),`${degrees(hot.m.casing_c)} casing · ${degrees(hot.m.winding_c)} winding`,tone(hot.h)):tile('Hottest motor','–','no motor readings','');
  const state=tile('Joint state',d.lowstate&&d.lowstate.hz!=null?`${d.lowstate.hz} Hz`:'–',d.lowstate&&d.lowstate.age_s!=null?`newest ${Math.round(d.lowstate.age_s*1000)} ms old`:(source.feeds.motors||{}).reason||'','');
  page.querySelector('.robot-summary').replaceChildren(battery,harness,hottestTile,state);
  page.querySelector('.robot-note').textContent=`Each bar runs to the limit the harness stops the robot at: ${limits.casing_c} °C for the casing, ${limits.winding_c} °C for the winding. Amber is ${Math.round(NEAR*100)}% of a limit or more.`;
  // One block per limb, a row per motor: casing and winding bars, each on its own limit.
  const motors=d.motors||[],limbs=page.querySelector('.robot-limbs');
  if(!motors.length){limbs.replaceChildren(node('p','','No motor readings yet: rt/lowstate has not arrived.'));return;}
  limbs.replaceChildren(...LIMBS.map(([name,from,to])=>{
    const block=node('section','limb'),head=node('div','limb-head');
    head.append(node('span','limb-name',name),node('span','limb-col','Casing'),node('span','limb-col','Winding'));
    block.append(head);
    for(const m of motors.slice(from,to)){
      const row=node('div','motor-row '+tone(heat(m,limits)));
      row.append(node('span','motor-name',short(m.name)));
      for(const [value,limit,label] of [[m.casing_c,limits.casing_c,'casing'],[m.winding_c,limits.winding_c,'winding']]){
        const cell=node('span','motor-cell '+(value==null?'':tone(value/limit))),bar=node('span','bar'),fill=node('span','bar-fill');
        fill.style.width=value==null?'0':`${Math.max(2,Math.min(100,value/limit*100))}%`;bar.append(fill);
        cell.title=`${m.name} ${label}: ${degrees(value)} of the ${limit} °C limit`;
        cell.append(bar,node('span','motor-value',degrees(value)));row.append(cell);
      }
      block.append(row);
    }
    return block;
  }));
}
