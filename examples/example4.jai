colliders: [..] *Physics.Collider;

polygon1, polygon2: Physics.Collider;

main :: () {
    #if OS == .WINDOWS {
        Windows :: #import "Windows";
        Windows.SetProcessDPIAware();
        Windows.timeBeginPeriod(1);
    }

    window = Window.create_window(window_width, window_height, "Physics");

    Simp.set_render_target(window, .LEFT_HANDED);

    W := cast(float) window_width;
    H := cast(float) window_height;
    C := Vector2.{ W / 2.0, H / 2.0 };
    O := 50.0;

    vertices1 := Vector2.[ .{280, 280}, .{400, 300}, .{300, 400} ];
    vertices2 := Vector2.[ .{500, 300}, .{600, 250}, .{700, 300}, .{750, 400}, .{500, 400} ];
    polygon1  = Physics.make_polygon(vertices1, 1.0, 0.5, 0.05);
    polygon2  = Physics.make_polygon(vertices2, 1.0, 0.5, 0.05);

    array_add(*colliders, *polygon1, *polygon2);

    quit := false;
    while !quit {
        current_time = seconds_since_init();
        dt := cast(float)(current_time - last_time);
        Clamp(*dt, 0, 0.1);
        last_time = current_time;

        mx, my := Window.get_mouse_pointer_position(right_handed = false);
        mouse_position := Vector2.{ cast(float) mx , cast(float) my };

        Input.update_window_events();

        for Input.get_window_resizes() {
            Simp.update_window(it.window);

            if it.window == window {
                window_width  = it.width;
                window_height = it.height;
            }
        }

        for Input.events_this_frame {
            if it.type == .QUIT then quit = true;
        }

        if Input.input_button_states[Input.Key_Code.ESCAPE] & .START {
            quit = true;
        }

        FORCE  :: 1000.0;
        TORQUE :: 5000.0;

        UP    :: Vector2.{  0, -1 };
        DOWN  :: Vector2.{  0,  1 };
        LEFT  :: Vector2.{ -1,  0 };
        RIGHT :: Vector2.{  1,  0 };

        if Input.input_button_states[Input.Key_Code.ARROW_UP] & .DOWN {
            Physics.add_force(*polygon1, FORCE * UP);
        }

        if Input.input_button_states[Input.Key_Code.ARROW_DOWN] & .DOWN {
            Physics.add_force(*polygon1, FORCE * DOWN);
        }

        ctrl_is_down := Input.input_button_states[Input.Key_Code.CTRL] & .DOWN;

        if Input.input_button_states[Input.Key_Code.ARROW_LEFT] & .DOWN {
            if ctrl_is_down Physics.add_torque(*polygon1, -TORQUE);
            else            Physics.add_force(*polygon1, FORCE * LEFT);
        }

        if Input.input_button_states[Input.Key_Code.ARROW_RIGHT] & .DOWN {
            if ctrl_is_down Physics.add_torque(*polygon1, TORQUE);
            else            Physics.add_force(*polygon1, FORCE * RIGHT);
        }

        draw_one_frame(dt);

        reset_temporary_storage();
        sleep_milliseconds(10);
    }
}

draw_one_frame :: (dt: float) {
    Simp.clear_render_target(.1, .1, .1, 1);

    Simp.set_shader_for_color(true);

    for colliders {
        LINEAR_FRICTION  :: 3.0;
        ANGULAR_FRICTION :: 3.0;

        force  := -it.linear_velocity  * it.mass    * LINEAR_FRICTION;
        torque := -it.angular_velocity * it.inertia * ANGULAR_FRICTION;

        Physics.add_force(it, force);
        Physics.add_torque(it, torque);
    }

    for colliders {
        Physics.update(it, dt);
    }

    for colliders {
        if it.kind == {
            case .CIRCLE;
                draw_circle(it.position, it.circle.radius, .{ 1, 1, 1, 1 }, 2.0);
                draw_segment(it.position, it.position + Vector2.{ cos(it.rotation), sin(it.rotation) } * it.circle.radius, .{ 1, 1, 1, 1 }, 2.0);

            case .POLYGON;
                draw_polygon(Physics.get_vertices(it.polygon), .{ 1, 1, 1, 1 }, 2.0);
        }
    }

/*
    for i: 0..colliders.count - 2 {
        for j: i + 1..colliders.count - 1 {
            contact, ok := make_contact(*colliders[i], *colliders[j]);

            draw_point(contact.start, .{ 0, 1, 0, 1}, 10.0);
            draw_point(contact.end  , .{ 1, 0, 0, 1}, 10.0);
        }
    }
*/

    vertices_a := Physics.get_vertices(polygon1.polygon);
    vertices_b := Physics.get_vertices(polygon2.polygon);

/*
    {
        sat_a, ok_a := find_sat_separation(vertices_a, vertices_b);
        sat_b, ok_b := find_sat_separation(vertices_a, vertices_b);

        if ok_a draw_point(sat_a.point, .{ 0, 1, 0, 1 }, 10.0);
        if ok_a draw_segment(sat_a.point, sat_a.point + sat_a.normal * sat_a.depth, .{ 0, 1, 0, 1 }, 2.0);

        if ok_b draw_point(sat_b.point, .{ 1, 0, 0, 1 }, 10.0);
        if ok_b draw_segment(sat_b.point, sat_b.point + sat_b.normal * sat_b.depth, .{ 1, 0, 0, 1 }, 2.0);
    }
*/

    {
        i := 0;
        j := (i + 1) % vertices_a.count;

        edge   := vertices_a[i] - vertices_a[j];
        vector := normalize(edge);
        perpendicular :: (using v: Vector2) -> Vector2 {
            return .{ -y, x };
        }
        normal := perpendicular(vector);

        draw_point(vertices_a[i], .{ 1, 0, 0, 1 }, 10.0);
        draw_segment(vertices_a[i], vertices_a[j], .{ 1, 0, 0, 1 }, 2.0);
        draw_segment(vertices_a[i], vertices_a[i] + normal * 20.0, .{ 0, 1, 0, 1 }, 2.0);

        support := Physics.find_sat_support(vertices_a[i], normal, vertices_b);
        draw_point(support.point, .{ 0, 1, 1, 1 }, 10.0);
        draw_segment(support.point, support.point + normal * support.depth, .{ 0, 1, 1, 1 }, 2.0);

    }

    Simp.swap_buffers(window);
}

draw_point :: (point: Vector2, color: Vector4, size: float) {
    s := size / 2.0;
    Simp.immediate_quad(point.x - s, point.y - s, point.x + s, point.y + s, color);
}

draw_segment :: (a: Vector2, b: Vector2, color: Vector4, width: float) {
    ab     := normalize(b - a);
    perp   := Vector2.{ ab.y, -ab.x };
    normal := perp * (width / 2.0);
    quad   := Vector2.[ a - normal, a + normal, b + normal, b - normal ];

    Simp.immediate_quad(quad[0], quad[1], quad[2], quad[3], color);
}

draw_polygon :: (vertices: [] Vector2, color: Vector4, width: float) {
    for vertices {
        a := vertices[it_index];
        b := vertices[(it_index + 1) % vertices.count];
        draw_segment(a, b, color, width);
    }
}

draw_circle :: (center: Vector2, radius: float, color: Vector4, width: float) {
    SEGMENT_COUNT :: 30;
    THETA         :: TAU / SEGMENT_COUNT;

    for i: 0..SEGMENT_COUNT - 1 {
        theta0 := THETA * i;
        theta1 := THETA * (i + 1);

        a := Vector2.{ center.x + cos(theta0) * radius, center.y + sin(theta0) * radius };
        b := Vector2.{ center.x + cos(theta1) * radius, center.y + sin(theta1) * radius };
        draw_segment(a, b, color, width);
    }
}

window        : Window.Window_Type;
window_width  : s32 = 1100;
window_height : s32 = 900;

current_time: float64;
last\  _time: float64;

#import "Basic";
#import "Math";

Window  :: #import "Window_Creation";
Input   :: #import "Input";
Simp    :: #import "Simp";
Physics :: #import "Physics";