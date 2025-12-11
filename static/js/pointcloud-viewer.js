// static/js/pointcloud-viewer.js
document.addEventListener('DOMContentLoaded', () => {

    const viewers = {}; // Stores each Three.js viewer instance
    let isSyncing = false; // Prevent recursive triggering during camera sync

    const createViewer = (containerId) => {
        const container = document.getElementById(containerId);
        if (!container) {
            console.error(`Container not found for ID: ${containerId}`);
            return null;
        }

        const scene = new THREE.Scene();
        const camera = new THREE.PerspectiveCamera(75, container.clientWidth / container.clientHeight, 0.1, 1000);
        const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
        renderer.setSize(container.clientWidth, container.clientHeight);
        container.appendChild(renderer.domElement);

        const controls = new THREE.TrackballControls(camera, renderer.domElement);
        controls.rotateSpeed = 2;
        controls.zoomSpeed = 2;
        controls.panSpeed = 2;
        controls.noZoom = false;
        controls.noPan = false;
        controls.staticMoving = true;       // Disable inertia for less drift
        controls.dynamicDampingFactor = 0.3;

        // Add ambient light (does not affect Points/MeshBasic, reserved for extension)
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
        scene.add(ambientLight);

        // Initial camera position
        camera.position.set(0, 0, 5);

        const animate = () => {
            requestAnimationFrame(animate);
            controls.update();
            renderer.render(scene, camera);
        };
        animate();

        window.addEventListener('resize', () => {
            camera.aspect = container.clientWidth / container.clientHeight;
            camera.updateProjectionMatrix();
            renderer.setSize(container.clientWidth, container.clientHeight);
        });

        const loadSceneData = async (url) => {
            try {
                // Clear the scene first (keep camera/controls/renderer unchanged)
                while (scene.children.length > 0) {
                    scene.remove(scene.children[0]);
                }
                // Re-add ambient light (was removed above)
                const ambient = new THREE.AmbientLight(0xffffff, 0.5);
                scene.add(ambient);

                const response = await fetch(url);
                if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
                return await response.json();
            } catch (error) {
                console.error(`Error loading scene data from ${url}:`, error);
                return null;
            }
        };

        const renderPoints = (positions, colors, pointSize = 0.03, vertexColors = true) => {
            if (!positions || positions.length === 0) return null;

            const geometry = new THREE.BufferGeometry();
            const flatPositions = new Float32Array(positions.flat());
            geometry.setAttribute('position', new THREE.BufferAttribute(flatPositions, 3));

            const flatColors = new Float32Array(colors.flat().map(c => c / 255.0));
            geometry.setAttribute('color', new THREE.BufferAttribute(flatColors, 3));

            const material = new THREE.PointsMaterial({
                size: pointSize,
                vertexColors,
                sizeAttenuation: true
            });
            return new THREE.Points(geometry, material);
        };

        const renderSpheres = (positions, colors, radius = 0.06, segments = 16) => {
            if (!positions || positions.length === 0) return null;

            const sphereGeometry = new THREE.SphereGeometry(radius, segments, segments);
            const material = new THREE.MeshBasicMaterial();
            const instancedMesh = new THREE.InstancedMesh(sphereGeometry, material, positions.length);

            const dummy = new THREE.Object3D();
            const instancedColors = new Float32Array(positions.length * 3);

            for (let i = 0; i < positions.length; i++) {
                dummy.position.set(positions[i][0], positions[i][1], positions[i][2]);
                dummy.updateMatrix();
                instancedMesh.setMatrixAt(i, dummy.matrix);

                instancedColors[i * 3] = colors[i][0] / 255.0;
                instancedColors[i * 3 + 1] = colors[i][1] / 255.0;
                instancedColors[i * 3 + 2] = colors[i][2] / 255.0;
            }
            instancedMesh.instanceColor = new THREE.BufferAttribute(instancedColors, 3);
            instancedMesh.instanceColor.needsUpdate = true;
            return instancedMesh;
        };

        const fitCameraToPoints = (geometry) => {
            if (!geometry.attributes.position) {
                console.warn("Geometry has no position attribute. Cannot fit camera.");
                return;
            }
            geometry.computeBoundingBox();
            const box = geometry.boundingBox;
            if (!box || !box.min || !box.max || box.isEmpty()) {
                console.warn("Bounding box is invalid. Skipping camera fit.");
                return;
            }
            const center = box.getCenter(new THREE.Vector3());
            const size = box.getSize(new THREE.Vector3());
            const maxDim = Math.max(size.x, size.y, size.z);
            const fov = camera.fov * (Math.PI / 180);
            let cameraZ = Math.abs(maxDim / 2 / Math.tan(fov / 2));
            cameraZ *= 1.5;

            camera.position.set(center.x, center.y, center.z + cameraZ);
            camera.lookAt(center);
            controls.target.copy(center);
            controls.update();
        };

        const renderScene = (data, isPredictionView = false) => {
            // Render background
            const backgroundPoints = renderPoints(data.background.positions, data.background.colors);
            if (backgroundPoints) scene.add(backgroundPoints);

            // Store mask and prompt objects
            const objects = {};

            data.masks.forEach(mask => {
                const maskPoints = renderPoints(mask.positions, mask.colors);
                if (maskPoints) {
                    maskPoints.visible = false; // Initially invisible
                    maskPoints.userData.maskId = mask.id;
                    objects[`mask-${mask.id}`] = maskPoints;
                    scene.add(maskPoints);
                }
            });

            if (isPredictionView && data.prompts) {
                data.prompts.forEach(prompt => {
                    const promptSpheres = renderSpheres(prompt.positions, prompt.colors);
                    if (promptSpheres) {
                        promptSpheres.visible = false; // Initially invisible
                        promptSpheres.userData.maskId = prompt.id;
                        objects[`prompt-${prompt.id}`] = promptSpheres;
                        scene.add(promptSpheres);
                    }
                });
            }

            // Fit camera to the scene's bounding box
            if (backgroundPoints) fitCameraToPoints(backgroundPoints.geometry);

            return objects;
        };

        return {
            loadAndRender: renderScene,
            loadData: loadSceneData,
            objects: {},
            camera,
            controls,
            renderer,
            _scene: scene,
            _renderPoints: renderPoints,
            _renderSpheres: renderSpheres,
        };
    };

    // Manage Pred-side state: selected click per object and overall visibility
    const selectedClickByObj = {};   // { [maskId]: 1|3|5 }
    const objVisible = {};           // { [maskId]: boolean } controlled by Obj button

    // Toggle visibility for all click geometries of an object; only keep selected one visible
    const applyVisibilityForObj = (maskId) => {
        const sel = selectedClickByObj[maskId] ?? 1;
        // Turn off all click geometries for this object
        [1, 3, 5].forEach(c => {
            const m = viewers.pred.objects[`mask-${maskId}-c${c}`];
            const p = viewers.pred.objects[`prompt-${maskId}-c${c}`];
            if (m) m.visible = false;
            if (p) p.visible = false;
        });
        // Turn on only the selected click if object is set to visible
        if (objVisible[maskId]) {
            const m = viewers.pred.objects[`mask-${maskId}-c${sel}`];
            const p = viewers.pred.objects[`prompt-${maskId}-c${sel}`];
            if (m) m.visible = true;
            if (p) p.visible = true;
        }
    };

    // Add all geometries for one click into Pred scene (no background, no camera reset)
    const addPredClickToScene = (predDataForClick, clickNum) => {
        if (!predDataForClick) return;
        // masks
        predDataForClick.masks.forEach(mask => {
            const pts = viewers.pred._renderPoints(mask.positions, mask.colors, 0.03, true);
            if (!pts) return;
            pts.visible = false; // Initially off, controlled by logic
            pts.userData.maskId = mask.id;
            pts.userData.click = clickNum;
            viewers.pred.objects[`mask-${mask.id}-c${clickNum}`] = pts;
            viewers.pred._scene.add(pts);
        });
        // prompts
        if (predDataForClick.prompts) {
            predDataForClick.prompts.forEach(prompt => {
                const spheres = viewers.pred._renderSpheres(prompt.positions, prompt.colors, 0.06, 16);
                if (!spheres) return;
                spheres.visible = false;
                spheres.userData.maskId = prompt.id;
                spheres.userData.click = clickNum;
                viewers.pred.objects[`prompt-${prompt.id}-c${clickNum}`] = spheres;
                viewers.pred._scene.add(spheres);
            });
        }
    };

    // Initialize viewer instances
    viewers.gt = createViewer('gt-pointcloud-container');
    viewers.pred = createViewer('pred-pointcloud-container');

    // === Camera synchronization (with re-entrancy lock to avoid recursive change) ===
    const syncCamera = (source, target) => {
        // Copy position, rotation, and target
        target.camera.position.copy(source.camera.position);
        target.camera.quaternion.copy(source.camera.quaternion);
        target.camera.up.copy(source.camera.up);
        target.controls.target.copy(source.controls.target);

        // Do not copy projectionMatrix; keep independent aspect ratios
        target.camera.updateProjectionMatrix();

        // Force update, but guarded by lock to avoid back-and-forth recursion
        target.controls.update();
        // Optionally render immediately for smoother feel
        // target.renderer.render(target.controls.object.parent ? target.controls.object.parent : new THREE.Scene(), target.camera);
    };

    const onGtChange = () => {
        if (isSyncing) return;
        isSyncing = true;
        syncCamera(viewers.gt, viewers.pred);
        isSyncing = false;
    };
    const onPredChange = () => {
        if (isSyncing) return;
        isSyncing = true;
        syncCamera(viewers.pred, viewers.gt);
        isSyncing = false;
    };

    viewers.gt.controls.addEventListener('change', onGtChange);
    viewers.pred.controls.addEventListener('change', onPredChange);
    // ==========================================

    // Controls Panel Logic
    const controlsPanel = document.getElementById('controls-panel');
    const updateControlsPanel = (maskIds) => {
        controlsPanel.innerHTML = '';
      
        // Outer grid: each object is a column
        const grid = document.createElement('div');
        grid.style.display = 'grid';
        grid.style.gridTemplateColumns = `repeat(${maskIds.length}, 1fr)`;
        grid.style.gap = '12px';
        grid.style.alignItems = 'center';
        grid.style.justifyItems = 'center';
        grid.style.width = '100%';
      
        // First row: Obj buttons
        maskIds.forEach((id, idx) => {
          const ob = document.createElement('a');
          ob.className = 'button is-small is-info';
          ob.style.minWidth = '90px';   // Fixed width, roughly equal to numbers row
          ob.textContent = `Obj ${idx + 1}`;
          ob.dataset.maskId = id;
      
          ob.addEventListener('click', (e) => {
            const i = parseInt(e.currentTarget.dataset.maskId, 10);
            e.currentTarget.classList.toggle('is-info');
            objVisible[i] = e.currentTarget.classList.contains('is-info');
      
            // Toggle GT visibility
            if (viewers.gt.objects[`mask-${i}`]) {
              viewers.gt.objects[`mask-${i}`].visible = objVisible[i];
            }
            // Toggle Pred visibility
            applyVisibilityForObj(i);
          });
      
          grid.appendChild(ob);
        });
      
        // Second row: click number buttons
        maskIds.forEach((id) => {
          const wrap = document.createElement('div');
          wrap.style.display = 'flex';
          wrap.style.gap = '6px';
      
          [1, 3, 5].forEach(num => {
            const cb = document.createElement('a');
            cb.className = 'button is-small is-light';
            cb.style.minWidth = '28px';  // Unified width for numbers
            cb.textContent = `${num}`;
            cb.dataset.maskId = id;
            cb.dataset.clickNum = num;
      
            if ((selectedClickByObj[id] ?? 1) === num) cb.classList.add('is-info');
      
            cb.addEventListener('click', (e) => {
              const i = parseInt(e.currentTarget.dataset.maskId, 10);
              const n = parseInt(e.currentTarget.dataset.clickNum, 10);
              selectedClickByObj[i] = n;
      
              [...wrap.querySelectorAll('a.button')].forEach(b => b.classList.remove('is-info'));
              e.currentTarget.classList.add('is-info');
              applyVisibilityForObj(i);
            });
      
            wrap.appendChild(cb);
          });
      
          grid.appendChild(wrap);
        });
      
        controlsPanel.appendChild(grid);
      };
      

    // ✅ Thumbnail selection logic
    const thumbs = document.querySelectorAll('.scene-thumb');

    thumbs.forEach(img => {
        img.addEventListener('click', () => {
            thumbs.forEach(t => t.classList.remove('selected'));
            img.classList.add('selected');
            loadScene(img.dataset.scene);
        });
    });

    // Default selection: Indoor 1
    const defaultThumb = document.querySelector('.scene-thumb[data-scene="Indoor/batch_0"]');
    if (defaultThumb) {
        defaultThumb.classList.add('selected');
        loadScene(defaultThumb.dataset.scene);
    }

    // ✅ loadScene function (migrated from old loadButton logic)
    async function loadScene(scenePath) {
        const [sceneType, batchDir] = scenePath.split('/');
        const basePath = `SNAP_Visualization/${sceneType}/${batchDir}`;

        const [backgroundData, gtData, pred1, pred3, pred5] = await Promise.all([
            viewers.gt.loadData(`${basePath}/background.json`),
            viewers.gt.loadData(`${basePath}/gt_mask.json`),
            viewers.pred.loadData(`${basePath}/click_1_data.json`),
            viewers.pred.loadData(`${basePath}/click_3_data.json`),
            viewers.pred.loadData(`${basePath}/click_5_data.json`)
        ]);

        if (!backgroundData || !gtData) {
            console.error('Background or GT data missing');
            return;
        }

        // Render GT
        const gtScene = { background: backgroundData, masks: gtData.masks, prompts: [] };
        viewers.gt.objects = viewers.gt.loadAndRender(gtScene, false);

        // Render Pred (background + camera)
        const predScene1 = { background: backgroundData, masks: [], prompts: [] };
        viewers.pred.objects = viewers.pred.loadAndRender(predScene1, true);

        addPredClickToScene(pred1, 1);
        addPredClickToScene(pred3, 3);
        addPredClickToScene(pred5, 5);

        const maskIds = gtData.masks.map(m => m.id).sort((a, b) => a - b);
        maskIds.forEach(id => {
            selectedClickByObj[id] = 5;
            objVisible[id] = true;
            if (viewers.gt.objects[`mask-${id}`]) {
                viewers.gt.objects[`mask-${id}`].visible = true;
            }
            applyVisibilityForObj(id);
        });

        updateControlsPanel(maskIds);
    }
});
