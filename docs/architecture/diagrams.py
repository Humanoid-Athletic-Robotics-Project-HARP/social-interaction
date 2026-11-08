"""SVG diagrams for the architecture PDF. Plain helpers, coordinates in a 700-wide frame."""
from html import escape

KINDS = {
    "ours":   ("#E1F5EE", "#0F6E56", "#085041", "#0F6E56"),
    "ext":    ("#EEEDFE", "#534AB7", "#3C3489", "#534AB7"),
    "apart":  ("#F1EFE8", "#888780", "#444441", "#5F5E5A"),
    "sim":    ("#FAEEDA", "#BA7517", "#633806", "#854F0B"),
    "robot":  ("#E6F1FB", "#185FA5", "#0C447C", "#185FA5"),
    "neutral": ("#FFFFFF", "#888780", "#2C2C2A", "#5F5E5A"),
    "data":   ("#FFFFFF", "#888780", "#2C2C2A", "#5F5E5A"),
}
EDGE = "#6b6a64"
MUTED = "#5F5E5A"


def box(x, y, w, h, title, lines=(), kind="ours"):
    fill, stroke, ink, sub = KINDS[kind]
    dash = ' stroke-dasharray="4 3"' if kind == "data" else ""
    out = [f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="7" fill="{fill}" '
           f'stroke="{stroke}" stroke-width="1"{dash}/>']
    block = 13 + 13.5 * len(lines)
    ty = y + (h - block) / 2 + 10.5
    cx = x + w / 2
    out.append(f'<text x="{cx}" y="{ty:.1f}" text-anchor="middle" font-size="12.5" '
               f'font-weight="600" fill="{ink}">{escape(title)}</text>')
    for i, line in enumerate(lines):
        out.append(f'<text x="{cx}" y="{ty + 15 + 13.5 * i:.1f}" text-anchor="middle" '
                   f'font-size="10.5" fill="{sub}">{escape(line)}</text>')
    return "".join(out)


def arrow(d, dashed=False, both=False, head=True):
    dash = ' stroke-dasharray="4 3"' if dashed else ""
    end = ' marker-end="url(#ah)"' if head else ""
    start = ' marker-start="url(#ah)"' if both else ""
    return f'<path d="{d}" fill="none" stroke="{EDGE}" stroke-width="1.1"{dash}{end}{start}/>'


def label(x, y, text, anchor="start", size=10, color=MUTED, weight=400, italic=False):
    st = ' font-style="italic"' if italic else ""
    return (f'<text x="{x}" y="{y}" text-anchor="{anchor}" font-size="{size}" '
            f'font-weight="{weight}" fill="{color}"{st}>{escape(text)}</text>')


def legend(y, items, x0=20):
    out, x = [], x0
    for kind, text in items:
        if kind == "dashed":
            out.append(f'<path d="M{x} {y} H{x + 22}" stroke="{EDGE}" stroke-width="1.1" stroke-dasharray="4 3"/>')
            out.append(label(x + 28, y + 3.5, text, size=10))
            x += 34 + 5.4 * len(text) + 18
            continue
        fill, stroke, _, _ = KINDS[kind]
        dash = ' stroke-dasharray="3 2"' if kind == "data" else ""
        out.append(f'<rect x="{x}" y="{y - 6}" width="12" height="12" rx="3" fill="{fill}" stroke="{stroke}"{dash}/>')
        out.append(label(x + 18, y + 3.5, text, size=10))
        x += 18 + 5.4 * len(text) + 18
    return "".join(out)


def svg(h, body, title):
    return (f'<svg viewBox="0 0 700 {h}" xmlns="http://www.w3.org/2000/svg" role="img" '
            f'aria-label="{escape(title)}" font-family="Helvetica Neue, Helvetica, Arial, sans-serif">'
            '<defs><marker id="ah" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6.5" '
            'markerHeight="6.5" orient="auto-start-reverse"><path d="M0 0L10 5L0 10z" '
            f'fill="{EDGE}"/></marker></defs>{body}</svg>')


def repo_map():
    b = [
        box(20, 20, 170, 60, "models/ (beside repo)", ["Holosoma recovery policy", "speech models"], "ext"),
        box(215, 20, 250, 60, "HIM harness + simulator",
            ["him-sim2real-management (beside repo)", "safety checks, MuJoCo, GR00T walker"], "ext"),
        box(490, 20, 190, 60, "Unitree G1 + PC2", ["lidar, camera, mic, DDS", "Jetson onboard computer"], "ext"),
        arrow("M190 50 H213"),
        '<rect x="20" y="120" width="660" height="250" rx="16" fill="none" stroke="#888780" stroke-width="1" stroke-dasharray="5 4"/>',
        label(36, 141, "submission/: image and host tools", size=11.5, color="#2C2C2A", weight=600),
        box(40, 160, 200, 66, "Host tools", ["build.sh · sim.sh · course.py", "stage_world · generate_world", "lidar_view · wave_check"]),
        box(260, 160, 200, 66, "Docker image", ["entrypoint.sh", "harp_node.py", "harp/ (29 modules)"]),
        box(480, 160, 180, 66, "Nav2 setup", ["nav2/ params + launch", "nav2_graceful_controller/", "(C++ Foxy port)"]),
        box(40, 270, 200, 66, "worlds/, routes/", ["19 MuJoCo courses", "pass rules in NAME.yaml", "waypoint routes"]),
        box(260, 270, 200, 66, "tests/", ["23 unit-test files", "run inside the image"]),
        arrow("M360 80 V158"),
        arrow("M440 160 V100 H585 V82"),
        label(592, 96, "runs on the robot", size=9.5),
        arrow("M240 185 H258"),
        arrow("M460 193 H478", both=True),
        arrow("M140 270 V228"),
        arrow("M360 270 V228"),
        box(20, 400, 124, 60, "speech/", ["listen.py, talk.py", "on a laptop"], "apart"),
        box(154, 400, 124, 60, "testground/", ["robot sensor tools", "runs on PC2"], "apart"),
        box(288, 400, 124, 60, "third_party/", ["YOLO26, BoT-SORT", "pinned submodules"], "apart"),
        box(422, 400, 124, 60, "context/", ["team memory notes", "check_memory.py"], "apart"),
        box(556, 400, 124, 60, "runs/ (gitignored)", ["172 scored runs", "from course.py"], "data"),
        arrow("M250 400 V212 H258", dashed=True),
        arrow("M216 460 V480 H692 V50 H682", dashed=True),
        label(470, 494, "copied to PC2 by testground/sync.sh", size=9.5),
        legend(522, [("ours", "our code, sim/robot path"), ("apart", "not wired into the image"),
                     ("ext", "outside this repo"), ("data", "generated"), ("dashed", "imports / copies")]),
    ]
    return svg(540, "".join(b), "Repository map")


def runtime():
    b = [
        box(170, 20, 360, 44, "G1_TASK_COMMAND  →  entrypoint.sh", (), "neutral"),
        arrow("M260 64 V87 H105 V108"), arrow("M305 64 V108"), arrow("M445 64 V87 H550 V108"),
        box(20, 110, 170, 64, "moves.py", ['{"to"} or {"moves"}', "walk · turn · curve"]),
        box(210, 110, 190, 64, "course_goal.py", ['a world, or {"route"}', "sends Nav2 goals"]),
        box(420, 110, 260, 64, "behavior.py + sim_people.py", ['{"behavior"}: wander, approach', "sim_people = stand-in detector"]),
        arrow("M305 174 V213"), arrow("M445 174 V213"),
        box(210, 215, 260, 64, "Nav2", ["NavFn planner · STVL costmaps", "Graceful Controller (our port)"]),
        box(490, 215, 190, 64, "lidar.py (sim)", ["MuJoCo rays + movers", "+ depth_camera.py"], "sim"),
        box(490, 305, 190, 64, "lidar_relay.py (robot)", ["Unitree lidar → ROS 2", "restamped, self-filtered"], "robot"),
        arrow("M490 247 H472"), arrow("M490 337 H472"),
        f'<path d="M481 247 V337" stroke="{EDGE}" stroke-width="1.1"/>',
        arrow("M340 279 V303"), label(348, 295, "/cmd_vel_nav", size=10),
        box(210, 305, 260, 64, "Command filter (one of)", ["approach_stop.py: default", "predictive_avoidance.py: opt-in"]),
        arrow("M340 369 V403"), label(348, 390, "/cmd_vel", size=10),
        arrow("M105 174 V403"), label(113, 292, "/cmd_vel", size=10),
        box(20, 405, 660, 64, "harp_node.py",
            ["/cmd_vel → GR00T walker (clipped to 0.6 m/s ahead, 0.4 sideways, 1 rad/s; zero after 0.5 s quiet)",
             "state.py republishes /odom /tf /joint_states /imu · wave.py sets the right arm"]),
        arrow("M300 469 V508"), label(308, 492, "joint targets", size=10),
        arrow("M420 510 V471", dashed=True), label(428, 492, "/safety/policy_state", size=10),
        box(20, 510, 660, 60, "HIM safety harness",
            ["checks every joint command · owns stand-up, stops and the motors",
             "MuJoCo simulator or the real G1, same image"], "ext"),
        legend(596, [("ours", "our process"), ("sim", "sim only"), ("robot", "robot only"), ("ext", "HIM (external)")]),
    ]
    return svg(614, "".join(b), "Processes inside the image")


def lifecycle():
    xs = [10, 149, 288, 427, 566]
    r1 = [("build.sh", ["runs HIM's walker", "prep, builds image", "→ submission.tar"], "ours"),
          ("course.py NAME", ["stage_world.py,", "a world image layer,", "then sim.sh"], "ours"),
          ("HIM's sim starts", ["MuJoCo + harness", "start our container"], "ext"),
          ("entrypoint.sh", ["starts lidar, Nav2,", "filter, goal sender,", "view_feed"], "ours"),
          ("nav2_ready.py", ["waits ≤ 15 s for", "Nav2 to configure"], "ours")]
    r2 = [("harp_node.py", ["arms HIM's safety", "checker, answers", "every state"], "ours"),
          ("Balance hold", ["HIM's GR00T balance", "~5 s, then hands", "control to us"], "ext"),
          ("control_ready", ["event on", "/harp/events"], "ours"),
          ("Driver starts", ["goal or move after", "SETTLE s; Nav2", "activates"], "ours"),
          ("Scoring", ["course.py checks", "events.jsonl against", "NAME.yaml → runs/"], "ours")]
    b = []
    for row, y in ((r1, 20), (r2, 150)):
        for i, (t, ls, k) in enumerate(row):
            b.append(box(xs[i], y, 124, 78, t, ls, k))
            if i < 4:
                b.append(arrow(f"M{xs[i] + 124} {y + 39} H{xs[i + 1] - 2}"))
    b.append(arrow("M628 98 V124 H72 V148"))
    b.append(legend(254, [("ours", "our code"), ("ext", "HIM's harness")]))
    return svg(270, "".join(b), "Life of one sim course run")


def tf_tree():
    b = [
        box(20, 62, 110, 44, "odom", ()),
        box(200, 62, 130, 44, "base_link", ()),
        box(410, 18, 140, 44, "livox_frame", ()),
        box(590, 18, 100, 44, "livox_rear", ()),
        box(410, 106, 230, 44, "d435_depth_optical_frame", ()),
        arrow("M130 84 H198"), label(164, 100, "leg odometry", anchor="middle", size=9.5),
        arrow("M330 84 H370 V40 H408"), arrow("M370 84 V128 H408", head=True),
        label(300, 168, "head frames: from joint angles + IMU", anchor="middle", size=9.5),
        arrow("M550 40 H588"), label(569, 76, "turned 180°", anchor="middle", size=9.5),
    ]
    return svg(178, "".join(b), "Transform tree published by state.py")


def behavior_loop():
    xs = [11, 151, 291, 431, 571]
    row = [("Wander", ["Nav2 goals ahead,", "kept within radius"]),
           ("Notice", ["still person on", "/harp/people"]),
           ("Approach", ["goal near them;", "camera checks legs"]),
           ("Walk up", ["one curve to", "STAND_OFF, facing"]),
           ("Talk", ["placeholder: stands", "until they leave"])]
    b = []
    for i, (t, ls) in enumerate(row):
        b.append(box(xs[i], 30, 118, 64, t, ls))
        if i < 4:
            b.append(arrow(f"M{xs[i] + 118} 62 H{xs[i + 1] - 2}"))
    b.append(label(420, 22, "confirmed", anchor="middle", size=9.5))
    b.append(box(291, 150, 118, 56, "Turn away", ["turn in place,", "then loop"]))
    b.append(arrow("M630 94 V178 H411"))
    b.append(arrow("M291 178 H60 V96"))
    b.append(arrow("M350 94 V124 H90 V96", dashed=True))
    b.append(label(220, 118, "never confirmed (a pillar): skipped", anchor="middle", size=9.5))
    return svg(222, "".join(b), "The behavior loop")


def filters():
    b = [
        label(20, 22, "approach_stop.py (default): stand still while something comes at the robot", size=11.5, color="#2C2C2A", weight=600),
        box(20, 36, 150, 60, "/cmd_vel_nav", ["from Nav2"], "neutral"),
        box(190, 36, 170, 60, "Strip ahead", ["nearest lidar point,", "outside the arms"]),
        box(380, 36, 150, 60, "Coming at it?", ["range shrinking faster", "than the robot walks"]),
        box(550, 36, 130, 60, "/cmd_vel", ["pass it on, or zero", "until clear"], "neutral"),
        arrow("M170 66 H188"), arrow("M360 66 H378"), arrow("M530 66 H548"),
        label(20, 140, "predictive_avoidance.py (opt-in): track what moves, predict it, choose a velocity", size=11.5, color="#2C2C2A", weight=600),
        box(20, 154, 115, 64, "Lidar scan", ["self-filtered"], "neutral"),
        box(150, 154, 140, 64, "obstacle_tracker", ["voxel tracks in odom", "with velocities"]),
        box(305, 154, 150, 64, "predictive_mpc", ["candidates over 4 s,", "3 velocity blocks"]),
        box(470, 154, 120, 64, "detour, make_way", ["waits out detours,", "steps aside if stuck"]),
        box(605, 154, 85, 64, "/cmd_vel", (), "neutral"),
        arrow("M135 186 H148"), arrow("M290 186 H303"), arrow("M455 186 H468"), arrow("M590 186 H603"),
        arrow("M220 218 V262 H303"),
        box(305, 242, 285, 40, "still points only → Nav2's costmaps", (), "neutral"),
        label(380, 248 + 52, "Nav2's own /cmd_vel_nav and plan are the reference the MPC stays close to", anchor="middle", size=9.5),
    ]
    return svg(312, "".join(b), "The two command filters")


def sensors():
    b = [
        label(175, 24, "In the sim", anchor="middle", size=11.5, color="#2C2C2A", weight=600),
        label(525, 24, "On the robot", anchor="middle", size=11.5, color="#2C2C2A", weight=600),
        box(20, 36, 310, 56, "HIM's sim state", ["true pose + 29 joint angles (/safety/policy_state)"], "ext"),
        box(370, 36, 310, 56, "Unitree lidar_driver", ["rt/utlidar/cloud_livox_mid360 on the robot's DDS"], "ext"),
        box(20, 122, 310, 64, "lidar.py + depth_camera.py", ["rays through a copy of the world, movers included", "MID-360 field of view, head shadow, noise"], "sim"),
        box(370, 122, 310, 64, "lidar_relay.py", ["read through Unitree's SDK (CycloneDDS)", "restamped with its arrival time"], "robot"),
        arrow("M175 92 V120"), arrow("M525 92 V120"),
        box(195, 216, 310, 56, "self_filter.py", ["drops points inside boxes around the robot's", "own body, posed from the joint angles"]),
        arrow("M175 186 V244 H193"), arrow("M525 186 V244 H507"),
        arrow("M350 272 V298"),
        box(100, 300, 500, 60, "Published clouds",
            ["/utlidar/cloud_livox_mid360 + a rear copy · /camera/depth/points (sim only)",
             "read by Nav2's STVL layers, the command filter and sim_people"], "neutral"),
        legend(384, [("sim", "sim only"), ("robot", "robot only"), ("ours", "shared"), ("ext", "source outside our code")]),
    ]
    return svg(400, "".join(b), "Where the lidar cloud comes from")


def observability():
    b = [
        box(20, 30, 150, 64, "Every process", ["events.py: one JSON", "line per event"]),
        box(190, 30, 140, 64, "Container output", ["+ /harp/events topic"], "neutral"),
        box(350, 30, 140, 64, "HIM's policy.log", ["the only way out", "of the container"], "ext"),
        box(510, 30, 170, 64, "course.py", ["splits it into", "events.jsonl + view.jsonl"]),
        arrow("M170 62 H188"), arrow("M330 62 H348"), arrow("M490 62 H508"),
        box(20, 136, 150, 56, "view_feed.py", ["cloud, pose, costmap", "a few times a second"]),
        arrow("M170 164 H260 V96"),
        box(330, 136, 170, 56, "Scoring", ["events.jsonl vs NAME.yaml"]),
        box(520, 136, 160, 56, "lidar_view.py", ["Rerun: cloud, costmap,", "plan, truth vs odom"]),
        arrow("M560 94 V114 H415 V134"), arrow("M640 94 V134"),
    ]
    return svg(206, "".join(b), "How a run is recorded and seen")


def speech():
    top = [(20, 120, "Sound in", ["mic · WAV ·", "robot mic (UDP)"], "neutral"),
           (155, 120, "Silero VAD", ["voice per 32 ms", "+ loudness check"], "ours"),
           (290, 125, "Smart Turn", ["has the person", "finished?"], "ours"),
           (430, 125, "Parakeet TDT", ["speech to text", "via sherpa-onnx"], "ours"),
           (570, 110, "Transcript", ["JSON line", "on stdout"], "neutral")]
    bot = [(570, 110, "talk.py", ["owns the", "conversation"], "ours"),
           (430, 125, "llama-server", ["Qwen3-4B, local,", "streamed"], "ours"),
           (290, 125, "Sentences", ["each spoken", "as soon as ready"], "ours"),
           (155, 120, "Kokoro voice", ["via sherpa-onnx"], "ours"),
           (20, 120, "Speaker", ["laptop for now"], "neutral")]
    b = [label(20, 26, "listen.py", size=11.5, color="#2C2C2A", weight=600),
         label(20, 250, "talk.py", size=11.5, color="#2C2C2A", weight=600)]
    for i, (x, w, t, ls, k) in enumerate(top):
        b.append(box(x, 36, w, 60, t, ls, k))
        if i < 4:
            b.append(arrow(f"M{x + w} 66 H{top[i + 1][0] - 2}"))
    for i, (x, w, t, ls, k) in enumerate(bot):
        b.append(box(x, 150, w, 60, t, ls, k))
        if i < 4:
            b.append(arrow(f"M{x} 180 H{bot[i + 1][0] + bot[i + 1][1] + 2}"))
    b.append(arrow("M625 96 V148"))
    b.append(label(633, 126, "pipe", size=9.5))
    b.append(arrow("M680 186 H692 V232 H80 V212", dashed=True))
    b.append(label(380, 224, "mute: stop listening while the robot speaks (UDP)", anchor="middle", size=9.5))
    return svg(260, "".join(b), "The speech pipeline")
