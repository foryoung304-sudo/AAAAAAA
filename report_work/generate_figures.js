const fs = require('fs');
const path = require('path');

const root = path.resolve(__dirname, '..');
const out = path.join(__dirname, 'figures');
fs.mkdirSync(out, { recursive: true });

function esc(s) { return String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;'); }
function svg(w, h, content) {
  return `<?xml version="1.0" encoding="UTF-8"?><svg xmlns="http://www.w3.org/2000/svg" width="${w}" height="${h}" viewBox="0 0 ${w} ${h}"><rect width="100%" height="100%" fill="white"/>${content}</svg>`;
}
function chart(title, xLabel, yLabel, series, file) {
  const w=960,h=550,l=90,r=30,t=70,b=75,pw=w-l-r,ph=h-t-b;
  const xs=series.flatMap(s=>s.data.map(p=>p[0])); const ys=series.flatMap(s=>s.data.map(p=>p[1]));
  let xmin=Math.min(...xs), xmax=Math.max(...xs), ymin=Math.min(...ys), ymax=Math.max(...ys);
  if (xmin===xmax) { xmin-=1; xmax+=1; } if (ymin===ymax) { ymin-=1; ymax+=1; }
  const pad=(ymax-ymin)*0.1; ymin-=pad; ymax+=pad;
  const X=x=>l+(x-xmin)/(xmax-xmin)*pw, Y=y=>t+ph-(y-ymin)/(ymax-ymin)*ph;
  let c=`<text x="${w/2}" y="35" text-anchor="middle" font-family="Arial" font-size="20" font-weight="bold" fill="#202938">${esc(title)}</text>`;
  for(let i=0;i<=5;i++){ const y=t+ph*i/5, v=ymax-(ymax-ymin)*i/5; c+=`<line x1="${l}" x2="${w-r}" y1="${y}" y2="${y}" stroke="#D8DEE8"/><text x="${l-10}" y="${y+5}" text-anchor="end" font-family="Arial" font-size="12" fill="#556070">${v.toFixed(1)}</text>`; }
  for(let i=0;i<=5;i++){ const x=l+pw*i/5, v=xmin+(xmax-xmin)*i/5; c+=`<line x1="${x}" x2="${x}" y1="${t}" y2="${t+ph}" stroke="#EEF1F5"/><text x="${x}" y="${h-b+25}" text-anchor="middle" font-family="Arial" font-size="12" fill="#556070">${v.toFixed(1)}</text>`; }
  c+=`<line x1="${l}" x2="${l}" y1="${t}" y2="${t+ph}" stroke="#657184" stroke-width="1.4"/><line x1="${l}" x2="${w-r}" y1="${t+ph}" y2="${t+ph}" stroke="#657184" stroke-width="1.4"/>`;
  series.forEach((s,i)=>{ const d=s.data.map((p,j)=>`${j?'L':'M'}${X(p[0]).toFixed(1)},${Y(p[1]).toFixed(1)}`).join(' '); c+=`<path d="${d}" fill="none" stroke="${s.color}" stroke-width="${s.width||2.3}" ${s.dash?`stroke-dasharray="${s.dash}"`:''}/><rect x="${l+15+i*205}" y="${t+10}" width="16" height="3" fill="${s.color}"/><text x="${l+37+i*205}" y="${t+15}" font-family="Arial" font-size="12" fill="#303947">${esc(s.name)}</text>`; });
  c+=`<text x="${w/2}" y="${h-15}" text-anchor="middle" font-family="Arial" font-size="14" fill="#303947">${esc(xLabel)}</text><text x="20" y="${h/2}" text-anchor="middle" transform="rotate(-90 20 ${h/2})" font-family="Arial" font-size="14" fill="#303947">${esc(yLabel)}</text>`;
  fs.writeFileSync(path.join(out,file), svg(w,h,c));
}
function readHeight() {
  const s=fs.readFileSync(path.join(root,'project','tools','height_log.txt'),'utf8').split(/\r?\n/);
  const start=s.findIndex(x=>x.startsWith('seq,')); const end=s.findIndex((x,i)=>i>start&&x.startsWith('HEIGHT_LOG_END'));
  const hdr=s[start].split(','); return s.slice(start+1,end).filter(x=>/^\d+,/.test(x)).map(x=>Object.fromEntries(hdr.map((k,i)=>[k,Number(x.split(',')[i])])));
}
const rows=readHeight(); const t0=rows[0].time_us;
const toPts=k=>rows.map(r=>[(r.time_us-t0)/1e6,r[k]]);
chart('Altitude-estimation log','Time (s)','Height (cm)',[
  {name:'ToF measurement', color:'#AAB3C2', width:1.1, data:toPts('tof_cm')},
  {name:'EKF height', color:'#165DAB', data:toPts('ekf_z_cm')},
  {name:'Control feedback', color:'#F08A24', data:toPts('final_height_cm')},
  {name:'Profile reference', color:'#566273', dash:'6 4', width:1.5, data:toPts('profile_height_cm')},
], 'fig_height_estimation.svg');
chart('Vertical-state and throttle log','Time (s)','Vertical velocity (cm/s)',[
  {name:'EKF vertical velocity', color:'#165DAB', data:toPts('ekf_vz_cm_s')},
  {name:'Profile vertical velocity', color:'#F08A24', dash:'6 4', width:1.5, data:toPts('profile_vz_cm_s')},
], 'fig_vertical_state.svg');
const debug=fs.readFileSync(path.join(root,'project','tools','debug_log_2.txt'),'utf8'); const vals=[...debug.matchAll(/roll=([-+\d.]+),\s*pitch=([-+\d.]+),yaw=([-+\d.]+)/g)].map(m=>[Number(m[1]),Number(m[2])]);
chart('Attitude samples from debug log','Debug sample index','Angle (deg)',[
  {name:'Roll',color:'#165DAB',data:vals.map((x,i)=>[i,x[0]])},{name:'Pitch',color:'#F08A24',data:vals.map((x,i)=>[i,x[1]])}
], 'fig_attitude_debug.svg');
const w=800,h=550,cx=390,cy=270,scale=22; let c=`<text x="400" y="38" text-anchor="middle" font-family="Arial" font-size="20" font-weight="bold" fill="#202938">Out-and-back position closure residual</text><line x1="90" y1="${cy}" x2="720" y2="${cy}" stroke="#738096"/><line x1="${cx}" y1="80" x2="${cx}" y2="450" stroke="#738096"/>`;
for(let v=-10;v<=10;v+=5){c+=`<text x="${cx+v*scale}" y="${cy+22}" text-anchor="middle" font-family="Arial" font-size="12">${v}</text><text x="${cx-10}" y="${cy-v*scale+4}" text-anchor="end" font-family="Arial" font-size="12">${v}</text>`;} c+=`<text x="400" y="510" text-anchor="middle" font-family="Arial" font-size="14">X residual (cm)</text><text x="25" y="270" text-anchor="middle" transform="rotate(-90 25 270)" font-family="Arial" font-size="14">Y residual (cm)</text>`;
const pts=[['Ideal return',0,0,'#303947'],['Body-frame residual',-1.43,-3.14,'#165DAB'],['Earth-frame residual',1.51,-9.42,'#F08A24']]; pts.forEach((p,i)=>{const x=cx+p[1]*scale,y=cy-p[2]*scale;c+=`<circle cx="${x}" cy="${y}" r="8" fill="${p[3]}"/><text x="${x+12}" y="${y-10}" font-family="Arial" font-size="13" fill="${p[3]}">${p[0]} (${p[1]}, ${p[2]})</text>`;}); fs.writeFileSync(path.join(out,'fig_position_closure.svg'),svg(w,h,c));
let a=`<text x="480" y="35" text-anchor="middle" font-family="Arial" font-size="20" font-weight="bold" fill="#202938">Flight-control software architecture</text>`; const boxes=[['IMU\\nICM42688',55,105,'#E9F2FF'],['ToF height\\nVL53L8 / DL1B',55,245,'#FFF1DF'],['Optical flow\\nLC302',55,385,'#EAF7EC'],['State estimation\\nMahony + EKF-lite',340,240,'#EEF1F5'],['Cascade control\\nposition / velocity / attitude',580,240,'#E9F2FF'],['Motor mixing\\nand ESC output',805,240,'#FFF1DF']]; boxes.forEach(b=>{a+=`<rect x="${b[1]}" y="${b[2]}" width="135" height="78" rx="10" fill="${b[3]}" stroke="#516072"/><text x="${b[1]+67}" y="${b[2]+30}" text-anchor="middle" font-family="Arial" font-size="13" fill="#273142">${b[0].replace('\\n','</text><text x="'+(b[1]+67)+'" y="'+(b[2]+52)+'" text-anchor="middle" font-family="Arial" font-size="13" fill="#273142">')}</text>`;}); [[190,144,340,270],[190,284,340,275],[190,424,340,290],[475,280,580,280],[715,280,805,280]].forEach(q=>a+=`<line x1="${q[0]}" y1="${q[1]}" x2="${q[2]}" y2="${q[3]}" stroke="#516072" stroke-width="2" marker-end="url(#arrow)"/>`); a=`<defs><marker id="arrow" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto"><path d="M0,0 L8,4 L0,8z" fill="#516072"/></marker></defs>${a}`; fs.writeFileSync(path.join(out,'fig_control_architecture.svg'),svg(960,520,a));
