/* ===================================================================
   CRX-30iA 双机三维场景（three.js r128）
   运动链参数直接取自 fanuc_crx30ia/crx30ia.urdf
   网格用 collision/*.stl（base64 内联于 crx30ia-meshes.js，scale 0.001）
   URDF 为 Z-up，整体挂在 rotation.x=-90° 的根组下转成 three 的 Y-up
   =================================================================== */
(function (global) {
  'use strict';

  /* URDF joint 链：origin(m) / axis / 关节名 */
  var CHAIN = [
    { nm: 'J1', org: [0, 0, 0.37], axis: [0, 0, 1], mesh: 'j1', mOrg: [0, 0, 0] },
    { nm: 'J2', org: [0, 0, 0], axis: [0, 1, 0], mesh: 'j2', mOrg: [0, 0, 0] },
    { nm: 'J3', org: [0, 0, 0.95], axis: [0, -1, 0], mesh: 'j3', mOrg: [0, 0, 0] },
    { nm: 'J4', org: [0, 0, 0], axis: [-1, 0, 0], mesh: 'j4', mOrg: [0, 0, 0] },
    { nm: 'J5', org: [0.75, 0, 0], axis: [0, -1, 0], mesh: 'j5', mOrg: [0, -0.185, 0] },
    { nm: 'J6', org: [0, -0.185, 0], axis: [-1, 0, 0], mesh: 'j6', mOrg: [0.18, 0, 0] }
  ];
  var FLANGE = [0.18, 0, 0];          /* J6_link → flange */
  var MESH_SCALE = 0.001;             /* STL 为 mm */

  function b64ToBuf(b64) {
    var bin = global.atob(b64), n = bin.length, u = new Uint8Array(n);
    for (var i = 0; i < n; i++) u[i] = bin.charCodeAt(i);
    return u.buffer;
  }

  /* 构建单台机器人，返回 {root, joints:{J1..J6}, flange} */
  function buildRobot(meshes, loader, mat) {
    var THREE = global.THREE;
    var root = new THREE.Group();
    root.name = 'crx30ia';

    function addMesh(parent, key, org) {
      if (!meshes[key]) return null;
      var geo = loader.parse(b64ToBuf(meshes[key]));
      geo.computeVertexNormals();
      var m = new THREE.Mesh(geo, mat);
      m.scale.setScalar(MESH_SCALE);
      m.position.set(org[0], org[1], org[2]);
      m.castShadow = true; m.receiveShadow = true;
      parent.add(m);
      return m;
    }

    addMesh(root, 'base', [0, 0, 0]);

    var parent = root, joints = {};
    CHAIN.forEach(function (j) {
      var g = new THREE.Group();
      g.name = j.nm;
      g.position.set(j.org[0], j.org[1], j.org[2]);
      g.userData.axis = new THREE.Vector3(j.axis[0], j.axis[1], j.axis[2]);
      parent.add(g);
      addMesh(g, j.mesh, j.mOrg);
      joints[j.nm] = g;
      parent = g;
    });

    var flange = new THREE.Group();
    flange.name = 'flange';
    flange.position.set(FLANGE[0], FLANGE[1], FLANGE[2]);
    parent.add(flange);

    root.userData.joints = joints;
    root.userData.flange = flange;
    return root;
  }

  /* 设置关节角（度） */
  function setJoints(robot, degs) {
    var joints = robot.userData.joints;
    Object.keys(degs).forEach(function (k) {
      var g = joints[k]; if (!g) return;
      g.setRotationFromAxisAngle(g.userData.axis, degs[k] * Math.PI / 180);
    });
  }

  /* ---------- 管道工件 ----------
     od/wt/len 单位 mm；轴线沿 X，中心抬到 hc 高度。
     由「外壁 + 内壁 + 两端环形端面」四片组成，端面用环形几何体呈现真实壁厚。 */
  function buildPipe(o) {
    var THREE = global.THREE;
    var od = (o.od || 300) / 1000, wt = (o.wt || 2) / 1000;
    var len = (o.len || 1400) / 1000, hc = (o.hc || 500) / 1000;
    var ro = od / 2, ri = ro - wt, seg = 96;

    var g = new THREE.Group();
    var mOut = new THREE.MeshStandardMaterial({
      color: 0xc8d4e2, metalness: 0.62, roughness: 0.38
    });
    var mIn = new THREE.MeshStandardMaterial({
      color: 0x7f8fa4, metalness: 0.5, roughness: 0.6, side: THREE.BackSide
    });
    var mEnd = new THREE.MeshStandardMaterial({
      color: 0xa8b6c8, metalness: 0.55, roughness: 0.45, side: THREE.DoubleSide
    });

    var outer = new THREE.Mesh(new THREE.CylinderGeometry(ro, ro, len, seg, 1, true), mOut);
    outer.castShadow = true; outer.receiveShadow = true;
    g.add(outer);
    g.add(new THREE.Mesh(new THREE.CylinderGeometry(ri, ri, len, seg, 1, true), mIn));

    /* 两端环形端面 */
    [-1, 1].forEach(function (s) {
      var ring = new THREE.Mesh(new THREE.RingGeometry(ri, ro, seg), mEnd);
      ring.rotation.x = Math.PI / 2;
      ring.position.y = s * len / 2;
      g.add(ring);
    });

    /* 中部环焊缝：略高于外壁的金色细环 */
    var weld = new THREE.Mesh(
      new THREE.TorusGeometry(ro, Math.max(wt * 0.7, 0.0016), 12, seg),
      new THREE.MeshStandardMaterial({ color: 0xe0a94a, metalness: 0.75, roughness: 0.3 })
    );
    weld.rotation.x = Math.PI / 2;
    g.add(weld);

    /* 轴线转到沿 Z（与机器人 Y 轴平行，两机跨管径对焊环缝），并抬到中心离地高度 */
    g.rotation.x = Math.PI / 2;
    g.position.y = hc;
    g.userData.weld = weld;
    g.userData.dims = { od: od, wt: wt, len: len, hc: hc };
    return g;
  }

  /* ---------- 场景 ---------- */
  function createScene(container, opts) {
    var THREE = global.THREE;
    opts = opts || {};
    var span = opts.span != null ? opts.span : 2.0;   /* 两机间距 (m) */

    var scene = new THREE.Scene();
    scene.background = null;
    scene.fog = new THREE.Fog(0x12274a, 8, 26);

    var w = container.clientWidth || 800, h = container.clientHeight || 500;
    var camera = new THREE.PerspectiveCamera(42, w / h, 0.05, 200);
    camera.position.set(3.6, 2.9, 4.6);

    var renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setPixelRatio(Math.min(global.devicePixelRatio || 1, 2));
    renderer.setSize(w, h);
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.domElement.style.cssText = 'display:block;width:100%;height:100%';
    container.appendChild(renderer.domElement);

    /* 灯光：主光 + 补光 + 半球环境 */
    scene.add(new THREE.HemisphereLight(0x9fc4f5, 0x16294a, 0.85));
    var key = new THREE.DirectionalLight(0xffffff, 0.9);
    key.position.set(4, 7, 3);
    key.castShadow = true;
    key.shadow.mapSize.set(2048, 2048);
    var sc = key.shadow.camera;
    sc.left = -5; sc.right = 5; sc.top = 5; sc.bottom = -5; sc.near = 0.5; sc.far = 22;
    scene.add(key);
    var fill = new THREE.DirectionalLight(0x7fb0f5, 0.35);
    fill.position.set(-4, 3, -3);
    scene.add(fill);

    /* 地面网格 + 承影面（管道模式场景更小，网格随之收紧） */
    var gsz = opts.mode === 'pipe' ? 6 : 16;
    var grid = new THREE.GridHelper(gsz, gsz * 2, 0x4f7fc4, 0x2c4d80);
    grid.material.opacity = 0.42; grid.material.transparent = true;
    scene.add(grid);
    var ground = new THREE.Mesh(
      new THREE.PlaneGeometry(gsz, gsz),
      new THREE.ShadowMaterial({ opacity: 0.3 })
    );
    ground.rotation.x = -Math.PI / 2;
    ground.position.y = -0.001;
    ground.receiveShadow = true;
    scene.add(ground);

    /* Z-up → Y-up 转换根 */
    var world = new THREE.Group();
    world.rotation.x = -Math.PI / 2;
    scene.add(world);

    var loader = new THREE.STLLoader();
    var matA = new THREE.MeshStandardMaterial({ color: 0xf0f2f5, metalness: 0.25, roughness: 0.55 });
    var matB = new THREE.MeshStandardMaterial({ color: 0xdfe6ef, metalness: 0.25, roughness: 0.55 });

    var meshes = global.CRX_MESHES || {};
    var robots = {};
    var pipe = null, fitRoot = world;
    var isPipeMode = opts.mode === 'pipe';

    /* 管道模式：只显示一段管道工件，不加载机器人 */
    if (isPipeMode) {
      pipe = buildPipe(opts.pipe || {});
      scene.add(pipe);
      fitRoot = pipe;
    } else {
    /* A 机在 -span/2，B 机在 +span/2 并绕 Z 转 180° 对向布置 */
    var A = buildRobot(meshes, loader, matA);
    A.position.set(-span / 2, 0, 0);
    world.add(A);
    robots.A = A;

    var B = buildRobot(meshes, loader, matB);
    B.position.set(span / 2, 0, 0);
    B.rotation.z = Math.PI;
    world.add(B);
    robots.B = B;

    /* 初始姿态：略微抬臂，避免完全竖直看不出关节 */
    setJoints(A, { J1: 0, J2: -25, J3: 40, J4: 0, J5: -20, J6: 0 });
    setJoints(B, { J1: 0, J2: -25, J3: 40, J4: 0, J5: -20, J6: 0 });

    /* 双机之间放置管道工件（若传入 opts.pipe） */
    if (opts.pipe) {
      pipe = buildPipe(opts.pipe);
      scene.add(pipe);
    }
    }

    /* 视图预设中心：管道模式对准管中心，双机模式对准工位中部 */
    var tgtY = isPipeMode ? pipe.position.y : (pipe ? pipe.position.y : 0.85);
    var camIso = isPipeMode ? [1.9, 1.35, 2.3] : [3.6, 2.9, 4.6];

    /* 轨道控制 */
    var controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.target.set(0, tgtY, 0);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;
    camera.position.set(camIso[0], camIso[1], camIso[2]);
    controls.minDistance = pipe ? 0.6 : 1.4;
    controls.maxDistance = 18;
    controls.maxPolarAngle = Math.PI / 2 - 0.03;
    controls.update();

    var raf = null;
    function loop() {
      raf = global.requestAnimationFrame(loop);
      controls.update();
      renderer.render(scene, camera);
    }
    loop();

    function resize() {
      var W = container.clientWidth, H = container.clientHeight;
      if (!W || !H) return;
      camera.aspect = W / H; camera.updateProjectionMatrix();
      renderer.setSize(W, H);
    }
    var ro = global.ResizeObserver ? new global.ResizeObserver(resize) : null;
    if (ro) ro.observe(container); else global.addEventListener('resize', resize);

    return {
      scene: scene, camera: camera, renderer: renderer, controls: controls,
      robots: robots, world: world,
      get pipe() { return pipe; },
      setJoints: function (k, d) { if (robots[k]) setJoints(robots[k], d); },
      /* 按工件参数重建管道 */
      setPipe: function (p) {
        if (!pipe) return;
        scene.remove(pipe);
        pipe.traverse(function (n) { if (n.geometry) n.geometry.dispose(); });
        pipe = buildPipe(p || {});
        scene.add(pipe);
        fitRoot = pipe;
        tgtY = pipe.position.y;
        controls.target.set(0, tgtY, 0); controls.update();
      },
      /* 焊缝显隐 */
      setWeldVisible: function (v) {
        if (pipe && pipe.userData.weld) pipe.userData.weld.visible = !!v;
      },
      /* 视图预设 */
      viewIso: function () { camera.position.set(camIso[0], camIso[1], camIso[2]); controls.target.set(0, tgtY, 0); controls.update(); },
      viewFit: function () {
        var box = new THREE.Box3().setFromObject(fitRoot);
        var c = box.getCenter(new THREE.Vector3()), s = box.getSize(new THREE.Vector3());
        var r = Math.max(s.x, s.y, s.z) * 0.5;
        var d = r / Math.tan(camera.fov * Math.PI / 360) * 1.5;
        camera.position.set(c.x + d * 0.6, c.y + d * 0.5, c.z + d * 0.75);
        controls.target.copy(c); controls.update();
      },
      dispose: function () {
        if (raf) global.cancelAnimationFrame(raf);
        if (ro) ro.disconnect(); else global.removeEventListener('resize', resize);
        renderer.dispose();
        if (renderer.domElement.parentNode) renderer.domElement.parentNode.removeChild(renderer.domElement);
      }
    };
  }

  global.CRXScene = { create: createScene, setJoints: setJoints, CHAIN: CHAIN };
})(window);
