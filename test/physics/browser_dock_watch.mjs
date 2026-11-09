import { fileURLToPath } from "node:url";
import { dirname, resolve } from "node:path";
const APP = process.env.OPERON_URL || "http://127.0.0.1:8841";
const REPO = resolve(dirname(fileURLToPath(import.meta.url)), "..", "..");
import WebSocket from "ws";
import { spawn } from "node:child_process";
import { rmSync } from "node:fs";
const PROF="/tmp/cr-prof-"+process.pid;
const CDP=9391;
const chrome=spawn("/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
 ["--headless=new","--disable-gpu","--no-sandbox","--no-first-run","--use-gl=angle","--use-angle=swiftshader",
  "--enable-unsafe-swiftshader",`--remote-debugging-port=${CDP}`,"--remote-allow-origins=*",
  `--user-data-dir=${PROF}`,"about:blank"],{stdio:"ignore"});
process.on("exit",()=>{try{chrome.kill("SIGKILL")}catch{};try{rmSync(PROF,{recursive:true,force:true})}catch{}});
const sleep=ms=>new Promise(r=>setTimeout(r,ms));
let t=null;
for(let i=0;i<60;i++){try{const l=await (await fetch(`http://127.0.0.1:${CDP}/json/list`)).json();t=l.find(x=>x.type==="page");if(t?.webSocketDebuggerUrl)break;}catch{} await sleep(250);}
const ws=new WebSocket(t.webSocketDebuggerUrl,{perMessageDeflate:false});
await new Promise(r=>ws.once("open",r));
let id=0;const pend=new Map();
ws.on("message",raw=>{const m=JSON.parse(raw.toString());if(m.id&&pend.has(m.id)){pend.get(m.id)(m);pend.delete(m.id);}});
const send=(me,pa={})=>new Promise(r=>{const i=++id;pend.set(i,m=>r(m.result));ws.send(JSON.stringify({id:i,method:me,params:pa}));});
const ev=async e=>(await send("Runtime.evaluate",{expression:e,returnByValue:true,awaitPromise:true}))?.result?.value;
await send("Runtime.enable");await send("Page.enable");
await send("Page.navigate",{url:`${APP}/teleop-studio.html`});
for(let i=0;i<90;i++){await sleep(400);const b=await ev(`document.getElementById("phys-badge")?.textContent||""`);if(b&&!/booting/i.test(b))break;}
await ev(`document.querySelector('#st-bar button[data-st="1"]').click()`);
await sleep(1500);
await ev(`document.getElementById("btn-auto").click()`);
const SLOT=[0.1,0.105224,0.072622];
console.log("   t(s)   cell [x,y,z]              d(slot)mm  held  gap    status");
for(let i=0;i<30;i++){
  await sleep(600);
  const d=await ev(`(()=>{const p=window.__operonRenderProbe?.();const e=p&&p[6];
    return JSON.stringify({c:e&&e.solved, g:document.getElementById("phys-gap")?.textContent,
      h:document.getElementById("phys-grip")?.textContent, s:document.getElementById("watch-status")?.textContent,
      v:document.getElementById("verdict-6")?.textContent});})()`);
  if(!d) continue;
  const o=JSON.parse(d); if(!o.c) continue;
  const dist=Math.hypot(o.c[0]-SLOT[0],o.c[1]-SLOT[1],o.c[2]-SLOT[2])*1000;
  console.log(`  ${((i+1)*0.6).toFixed(1).padStart(5)}  [${o.c.map(v=>v.toFixed(3)).join(",")}]  ${dist.toFixed(1).padStart(8)}  ${/holding/.test(o.h)?"yes":"no "}  ${String(o.g).padEnd(8)} ${String(o.s).slice(0,34)}`);
}
console.log("final verdict:", await ev(`document.getElementById("verdict-6")?.textContent`));
process.exit(0);
