percentage_loop: float64;
finish_time: float64;

camera: Camera3;
mesh: Mesh;

UPDATE_RATE_MS :: 16;

world: CbtWorldHandle;

sphere_shape: CbtShapeHandle;
sphere_body: CbtBodyHandle;

sphere_mesh: Mesh;

main :: () {
    koda_init();

    create_window("Fat Goblins", 1920, 1080);
    defer destroy_window();

    set_mouse_capture(true);
    set_target_fps(0);
    set_vsync(false);

    camera.position = .{25, 15, 10};
    
    yaw_quaternion: Quaternion;
    set_from_axis_and_angle(*yaw_quaternion, .{0, 1, 0}, -2.0);
    pitch_quaternion: Quaternion;
    set_from_axis_and_angle(*pitch_quaternion, .{1, 0, 0}, 0.0);
    camera.rotation = pitch_quaternion * camera.rotation * yaw_quaternion;

    mesh.geometry = load_or_save_serialized_geometry("assets/sample_town.geom", "assets/sample_town.obj");
    add_material(*mesh, load_texture("assets/sample_town.png"));

    sphere_mesh.geometry = create_icosphere(1.0, 4);
    add_material(*sphere_mesh, load_texture("assets/sample_town.png"));

    world = cbtWorldCreate();
    defer cbtWorldDestroy(world);

    cbtWorldSetGravity(world, .{0, -15, 0});

    sphere_shape = cbtShapeAllocate(CBT_SHAPE_TYPE_SPHERE);
    defer cbtShapeDeallocate(sphere_shape);
    cbtShapeSphereCreate(sphere_shape, 1.0);
    defer cbtShapeDestroy(sphere_shape);

    sphere_body = cbtBodyAllocate();
    defer cbtBodyDeallocate(sphere_body);
    // its a sideways matrix so the last vector is the position I guess
    cbtBodyCreate(sphere_body, 10.0, .[.{1, 0, 0}, .{0, 1, 0}, .{0, 0, 1}, .{5, 25, 5}], sphere_shape);
    cbtBodySetRestitution(sphere_body, 0.5);
    cbtBodySetRollingFriction(sphere_body, 0.0);
    cbtBodySetSpinningFriction(sphere_body, 0.0);
    // cbtBodySetFriction(sphere_body, 0.0);

    cbtWorldAddBody(world, sphere_body);
    
    level_shape := cbtShapeAllocate(CBT_SHAPE_TYPE_TRIANGLE_MESH);
    defer cbtShapeDeallocate(level_shape);
    cbtShapeTriMeshCreateBegin(level_shape);
    defer cbtShapeTriMeshDestroy(level_shape);

    tightly_packed_vertices := NewArray(mesh.geometry.vertices.count, Vector3);
    defer array_free(tightly_packed_vertices);
    for i: 0..tightly_packed_vertices.count - 1 {
        tightly_packed_vertices[i] = mesh.geometry.vertices[i].position;
    }

    cbtShapeTriMeshAddIndexVertexArray(
        level_shape,
        cast(s32) mesh.geometry.vertex_indices.count / 3,
        mesh.geometry.vertex_indices.data,
        3 * size_of(u32),
        cast(s32) tightly_packed_vertices.count,
        tightly_packed_vertices.data,
        size_of(Vector3));

    cbtShapeTriMeshCreateEnd(level_shape);

    level_body := cbtBodyAllocate();
    defer cbtBodyDeallocate(level_body);
    cbtBodyCreate(level_body, 0.0, .[.{1, 0, 0}, .{0, 1, 0}, .{0, 0, 1}, .{0, 0, 0}], level_shape);
    cbtBodySetRestitution(level_body, 1.0);

    cbtWorldAddBody(world, level_body);

    last_time := get_time() * 1000;
    next_physics_time := last_time;
    while !should_exit() {
        new_time := get_time() * 1000;
        percentage_loop = (finish_time - last_time) / (new_time - last_time);
        last_time = new_time;

        while new_time >= next_physics_time {
            update(UPDATE_RATE_MS);

            cbtWorldStepSimulation(world, 1.0 / 60.0, 4, 1.0 / 60.0);

            next_physics_time += UPDATE_RATE_MS;
        }

        render();
        
        reset_temporary_storage();

        finish_time = get_time() * 1000;
    }
}

update :: (dt: float) {
    update_inputs();

    forward := get_forward_vector(camera);
    right := get_right_vector(camera);
    up := Vector3.{0, 1, 0};

    accel: Vector3;
    if input.keys[Key.W] {
        accel += forward;
    }
    if input.keys[Key.S] {
        accel -= forward;
    }
    if input.keys[Key.A] {
        accel -= right;
    }
    if input.keys[Key.D] {
        accel += right;
    }
    if input.keys[Key.SPACE] {
        accel += up;
    }
    if input.keys[Key.LEFT_CONTROL] {
        accel -= up;
    }

    accel /= 10.0;
    if input.keys[Key.LEFT_SHIFT] {
        accel *= 10.0;
    }

    camera.position += accel;
    
    yaw_quaternion: Quaternion;
    set_from_axis_and_angle(*yaw_quaternion, .{0, 1, 0}, xx input.mouse_delta_x / 1000.0);
    
    pitch_quaternion: Quaternion;
    set_from_axis_and_angle(*pitch_quaternion, .{1, 0, 0}, xx input.mouse_delta_y / 1000.0);

    camera.rotation = pitch_quaternion * camera.rotation * yaw_quaternion;

    koda.transform = get_transform(camera);
}

render :: () {
    clear(.{0, 0, 0, 0});

    draw_mesh(mesh);
    
    position: Vector3;
    cbtBodyGetCenterOfMassPosition(sphere_body, *position);
    draw_mesh(sphere_mesh, position);

    draw_fps(10, 10);
    draw_text(tprint("CPU: %1%%", round(percentage_loop * 10000.0) / 100.0), .{10, 40});
}

round :: (value: float) -> float {
    return floor(value + 0.5);
}

round :: (value: float64) -> float64 {
    return floor(value + 0.5);
}

#import "Basic";
#import "Math";

#import "KodaJai";
#import "JaiBullet";