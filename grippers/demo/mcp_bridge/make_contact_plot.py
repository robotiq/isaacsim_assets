"""
Live gripper pad contact-force plot, overlaid in the running Isaac Sim viewport.

Draws a "Gripper Pad Contact Force" window with two traces — left and right
inner-finger (pad) force in Newtons against whatever object is being gripped —
scrolling over the last ~500 physics frames with an auto-scaling y-axis.

Run it from a host shell (NOT inside Isaac) while a session with the MCP
extension is up (see README.md); it connects to the extension's TCP socket on
127.0.0.1:8766 and injects the plot code via `simulation.execute_script`.

    python3 make_contact_plot.py      # create / recreate the plot
    python3 remove_contact_plot.py    # tear it down

Alongside the plot it also UDP-broadcasts the two raw pad forces as an ASCII
"fL fR" datagram to 127.0.0.1:8770 every physics frame, so host-side consumers
(e.g. the DualSense trigger-force-feedback node) can drive haptics from the
exact same signal. Broadcasting to a port with no listener is a harmless no-op.

Works on BOTH physics backends; the backend is detected once at creation:
  * Newton (mujoco_warp): reads per-contact `efc.force`. Re-attaches to the live
    solver Data on a 2 s throttle so it survives an Isaac Stop -> Play.
  * PhysX: applies `PhysxContactReportAPI` to the two inner fingers and reads
    `get_physx_simulation_interface().get_contact_report()` each frame, summing
    impulse / physics_dt for pad-vs-external-object contacts.

Caveats:
  * PhysX contact reporting only activates after a Stop -> Play resync; if the
    forces read 0 during a firm grasp, stop and replay once.
  * The prim paths and Newton geom/joint indices below are specific to the
    UR5e + 2F-85 teleop scene (ur5robot_with_2F-85.usda). Adjust for other scenes.
  * Never scan the Python heap per frame — an earlier version fell back to
    `gc.get_objects()` every frame when it could not find the Newton solver,
    which dropped the sim from ~50 to ~2.6 FPS. Backend detection is one-shot.
"""
import json, socket
def send(cmd, t=70):
    s=socket.socket(); s.settimeout(t); s.connect(("127.0.0.1",8766))
    s.sendall(json.dumps(cmd).encode()); buf=b""
    while True:
        c=s.recv(65536)
        if not c: break
        buf+=c
        try: return json.loads(buf.decode())
        except: continue
    return {}
code=r'''
import traceback
try:
    import omni.ui as ui, omni.kit.app, carb, gc, time, socket, numpy as np
    from collections import deque

    # Host-side consumers (e.g. the DualSense trigger-force-feedback node) read
    # the raw pad forces off this UDP stream. Sending to a dead port is a no-op.
    UDP_ADDR=("127.0.0.1", 8770)

    # ------------------------------------------------------------------ paths
    ROBOT_ROOT="/World/ur5e"
    # The 2F-85 fingertip pad is now its own rigid body
    # (left/right_fingertip), split out of inner_finger, so the grasp contact
    # (and its force) lands on the fingertip body — watch that, not inner_finger.
    LEFT_PAD ="/World/ur5e/wrist_3_link/Robotiq_2F_85_edit/Robotiq_2F_85/left_fingertip"
    RIGHT_PAD="/World/ur5e/wrist_3_link/Robotiq_2F_85_edit/Robotiq_2F_85/right_fingertip"
    SCAN_INTERVAL=2.0      # Newton: re-find the live Data at most this often (s)

    def A(x):
        try: return np.asarray(x.numpy())
        except: return np.asarray(x)
    def flat(x): return np.asarray(A(x)).reshape(-1)

    # ------------------------------------------------------- NEWTON discovery
    def find_live():
        """Return (Model, live Data) selecting the mujoco_warp Data whose qpos
        has no NaN (a stop->play leaves a stale corrupted Data lingering)."""
        models=[]; datas=[]
        for o in gc.get_objects():
            try: m=type(o).__module__; n=type(o).__name__
            except: continue
            if isinstance(m,str) and "mujoco_warp" in m:
                if n=="Model": models.append(o)
                elif n=="Data": datas.append(o)
        live=None
        for d in datas:
            try:
                if not np.isnan(np.asarray(d.qpos.numpy())).any(): live=d; break
            except: pass
        if live is None and datas: live=datas[0]
        return (models[0] if models else None), live

    OBJ_GEOMS={22,23}; OBJ_BODIES={22,23}
    def newton_seed(st):
        mjm,mjd=find_live()
        st["mjm"]=mjm; st["mjd"]=mjd
        if mjm is not None:
            gb=flat(mjm.geom_bodyid).astype(int); jb=flat(mjm.jnt_bodyid).astype(int)
            st["gb"]=gb; st["par"]=flat(mjm.body_parentid).astype(int)
            st["lseed"]={int(jb[j]) for j in (9,10,11,12)}
            st["rseed"]={int(jb[j]) for j in (15,16,17,18)}
    def newton_side(st,b):
        c=int(b); par=st["par"]
        for _ in range(24):
            if c in st["lseed"]: return "L"
            if c in st["rseed"]: return "R"
            if c<=0: break
            c=int(par[c])
        return None
    def read_newton(st):
        mjm,mjd=st["mjm"],st["mjd"]
        if mjm is None or mjd is None: return None
        nacon=int(flat(mjd.nacon)[0]); fL=0.0; fR=0.0
        if nacon>0:
            geom=np.asarray(mjd.contact.geom.numpy()); ea=np.asarray(mjd.contact.efc_address.numpy())
            ef=flat(mjd.efc.force); gb=st["gb"]
            for k in range(nacon):
                g0,g1=int(geom[k,0]),int(geom[k,1])
                if g0 in OBJ_GEOMS: other=g1
                elif g1 in OBJ_GEOMS: other=g0
                else: continue
                ob=int(gb[other])
                if ob==0 or ob in OBJ_BODIES: continue
                s=newton_side(st,ob)
                if s is None: continue
                addr=int(ea[k,0])
                if 0<=addr<ef.size:
                    f=abs(float(ef[addr]))
                    if s=="L": fL+=f
                    else: fR+=f
        return fL,fR

    # -------------------------------------------------------- PHYSX discovery
    def physx_setup(st):
        import omni.physx, omni.usd
        from pxr import PhysxSchema
        stage=omni.usd.get_context().get_stage()
        newly=False
        for p in (LEFT_PAD,RIGHT_PAD):
            prim=stage.GetPrimAtPath(p)
            if not prim.HasAPI(PhysxSchema.PhysxContactReportAPI): newly=True
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        # physics dt
        dt=1.0/60.0
        for p in stage.Traverse():
            if p.GetTypeName()=="PhysicsScene":
                sc=PhysxSchema.PhysxSceneAPI.Get(stage,p.GetPath())
                a=sc.GetTimeStepsPerSecondAttr()
                if a and a.Get(): dt=1.0/float(a.Get())
                break
        st["si"]=omni.physx.get_physx_simulation_interface(); st["dt"]=dt
        from pxr import PhysicsSchemaTools
        st["_p"]=lambda e:(str(PhysicsSchemaTools.intToSdfPath(e)))
        return newly
    def read_physx(st):
        import math
        si=st["si"]; dt=st["dt"]; P=st["_p"]
        hdrs,data=si.get_contact_report()
        fL=0.0; fR=0.0
        for h in hdrs:
            a0=P(h.actor0); a1=P(h.actor1)
            pad=None; other=None
            if LEFT_PAD in (a0,a1): pad="L"
            elif RIGHT_PAD in (a0,a1): pad="R"
            else: continue
            other = a1 if (LEFT_PAD in a0 or RIGHT_PAD in a0) else a0
            if other.startswith(ROBOT_ROOT): continue    # ignore self/arm contacts
            off=h.contact_data_offset; n=h.num_contact_data; mag=0.0
            for i in range(off,off+n):
                if i<len(data):
                    im=data[i].impulse; mag+=math.sqrt(im[0]**2+im[1]**2+im[2]**2)
            f=mag/dt
            if pad=="L": fL+=f
            else: fR+=f
        return fL,fR

    # ------------------------------------------------------------ teardown old
    prev=getattr(carb,"_contactplot",None)
    if prev is not None:
        try: prev["sub"]=None
        except: pass
        try: prev["win"].destroy()
        except: pass
        try: prev["udp"].close()
        except: pass

    # -------------------------------------------------- backend detect (once)
    st={"smax":10.0,"backend":None,"last_scan":0.0,"mjm":None,"mjd":None}
    newton_seed(st)
    if st["mjd"] is not None:
        st["backend"]="newton"; note="Newton (mujoco_warp efc.force)"
    else:
        st["backend"]="physx"; newly=physx_setup(st)
        note="PhysX (contact report)"
        if newly: note+=" - APPLIED reporting; do ONE stop->play if forces read 0"
    print("backend:", st["backend"])

    def nice_ceil(x):
        import math
        if x<=1e-6: return 10.0
        e=math.floor(math.log10(x)); b=x/(10**e)
        for m in (1,2,5,10):
            if b<=m: return m*(10**e)
        return 10**(e+1)

    udp=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    st["udp"]=udp

    N=500
    bL=deque([0.0]*N,maxlen=N); bR=deque([0.0]*N,maxlen=N)

    win=ui.Window("Gripper Pad Contact Force", width=760, height=400)
    ref={"ticks":[]}
    with win.frame:
        with ui.VStack(spacing=3):
            with ui.HStack(height=20, spacing=18):
                for lbl,col in [("left pad [N]",0xffffff00),("right pad [N]",0xffff40ff)]:
                    with ui.HStack(width=0,spacing=5):
                        ui.Rectangle(width=16,height=14,style={"background_color":col})
                        ui.Label(lbl,width=0,style={"color":col})
                ui.Spacer()
                ui.Label("backend: "+note, width=0, style={"color":0xff66ccff})
            with ui.HStack(height=270):
                with ui.VStack(width=44):
                    for i in range(5):
                        lb=ui.Label("",height=14,alignment=ui.Alignment.RIGHT_TOP,style={"color":0xffbbbbbb})
                        ref["ticks"].append(lb)
                        if i<4: ui.Spacer()
                with ui.ZStack():
                    ui.Rectangle(style={"background_color":0xff141414})
                    with ui.VStack():
                        for i in range(5):
                            ui.Rectangle(height=1,style={"background_color":0x40ffffff})
                            if i<4: ui.Spacer()
                    ref["L"]=ui.Plot(ui.Type.LINE,0.0,st["smax"],*([0.0]*N),style={"color":0xffffff00,"background_color":0x00000000})
                    ref["R"]=ui.Plot(ui.Type.LINE,0.0,st["smax"],*([0.0]*N),style={"color":0xffff40ff,"background_color":0x00000000})
            ref["val"]=ui.Label("left = --   right = --", height=16)
            ui.Label("x: last ~500 frames | y auto-scales | Newton re-attaches after replay",height=14,style={"color":0xff888888})

    def set_ticks(smax):
        for i,lb in enumerate(ref["ticks"]): lb.text=f"{smax*(1-i/4.0):.0f}"
    set_ticks(st["smax"])

    def on_update(e):
        # NEWTON only: re-find the live Data on a throttle (survives stop->play).
        # Never scan the heap per-frame -- that is what tanked FPS before.
        if st["backend"]=="newton":
            now=time.time()
            if now-st["last_scan"]>SCAN_INTERVAL:
                st["last_scan"]=now
                if st["mjd"] is None: newton_seed(st)
            try: res=read_newton(st)
            except Exception:
                st["mjd"]=None; return    # stale after replay; throttled reseed will fix
        else:
            try: res=read_physx(st)
            except Exception: return
        if res is None: return
        fL,fR=res
        try: udp.sendto(("%.4f %.4f" % (fL,fR)).encode(), UDP_ADDR)
        except Exception: pass
        bL.append(fL); bR.append(fR)
        peak=max(max(bL),max(bR)); target=nice_ceil(max(10.0,peak*1.2))
        if abs(target-st["smax"])>1e-6:
            st["smax"]=target
            try: ref["L"].scale_max=target; ref["R"].scale_max=target
            except: pass
            set_ticks(target)
        ref["L"].set_data(*bL); ref["R"].set_data(*bR)
        ref["val"].text=f"left = {fL:7.2f} N   right = {fR:7.2f} N   (y-max {st['smax']:.0f})"

    sub=omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(on_update, name="contact_force_plot")
    carb._contactplot={"win":win,"sub":sub,"udp":udp}
    print("OK: contact-force plot recreated (backend="+st["backend"]+")")
except Exception:
    print("ERR:\n"+traceback.format_exc())
'''
r=send({"type":"simulation.execute_script","params":{"code":code}})
out=r.get("result",{})
print("\n".join(l for l in out.get("stdout","").splitlines() if not l.startswith("[mcp]")))
if out.get("stderr"): print("STDERR:", out["stderr"][-800:])
