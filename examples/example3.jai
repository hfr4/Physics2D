colliders: [..] *Physics.Collider;
poly1, poly2, circ1, circ2: Physics.Collider;

main :: () {
    #if OS == .WINDOWS {
        Windows :: #import "Windows";
        Windows.SetProcessDPIAware();
        Windows.timeBeginPeriod(1);
    }

    window = Window.create_window(window_width, window_height, "Physics");

    Simp.set_render_target(window, .LEFT_HANDED);

    my_font = Simp.get_font_at_size("assets", "OpenSans.ttf", 22);
    assert(my_font != null);

    circ1 = Physics.make_circle(.{600, 325}, 50.0, 1.0, 0.5, 0.05);
    circ2 = Physics.make_circle(.{700, 650}, 100.0, 1.0, 0.5, 0.05);
    poly1 = Physics.make_polygon(.[ .{100, 100}, .{200, 100}, .{200, 200}, .{100, 200} ], 1.0, 0.5, 0.05);
    poly2 = Physics.make_polygon(.[ .{200, 600}, .{300, 550}, .{400, 600}, .{450, 700}, .{200, 700} ], 1.0, 0.5, 0.05);

    array_add(*colliders, *poly1, *poly2, *circ1, *circ2);

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
            Physics.add_force(*poly1, FORCE * UP);
            Physics.add_force(*circ1, FORCE * UP);
        }

        if Input.input_button_states[Input.Key_Code.ARROW_DOWN] & .DOWN {
            Physics.add_force(*poly1, FORCE * DOWN);
            Physics.add_force(*circ1, FORCE * DOWN);
        }

        ctrl_is_down := Input.input_button_states[Input.Key_Code.CTRL] & .DOWN;

        if Input.input_button_states[Input.Key_Code.ARROW_LEFT] & .DOWN {
            if ctrl_is_down Physics.add_torque(*poly1, -TORQUE);
            else            Physics.add_force(*poly1, FORCE * LEFT);
            if ctrl_is_down Physics.add_torque(*circ1, -TORQUE);
            else            Physics.add_force(*circ1, FORCE * LEFT);
        }

        if Input.input_button_states[Input.Key_Code.ARROW_RIGHT] & .DOWN {
            if ctrl_is_down Physics.add_torque(*poly1, TORQUE);
            else            Physics.add_force(*poly1, FORCE * RIGHT);
            if ctrl_is_down Physics.add_torque(*circ1, TORQUE);
            else            Physics.add_force(*circ1, FORCE * RIGHT);
        }

        draw_one_frame(dt);

        reset_temporary_storage();
        sleep_milliseconds(10);
    }
}

draw_one_frame :: (dt: float) {
    mx, my := Window.get_mouse_pointer_position(right_handed = false);
    mouse_position := Vector2.{ cast(float) mx , cast(float) my };

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

    contacts: [..] Physics.Contact;
    contacts.allocator = temp;
    for i: 0..colliders.count - 2 {
        for j: i + 1..colliders.count - 1 {
            c, count := Physics.generate_contacts(colliders[i], colliders[j]);
            for 0..count - 1 array_add(*contacts, c[it]);
        }
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

    for contacts {
        wa := Physics.get_position(it.anchor_A);
        wb := Physics.get_position(it.anchor_B);

        draw_segment(wa, wa + it.normal * it.depth, .{ .4, 1, 0, 1 }, 2.0);

        Simp.set_shader_for_text();

        Simp.draw_text(my_font, xx wa.x, xx (wa.y - 20), "A", .{ 1., .4, 0, 1 });
        Simp.draw_text(my_font, xx wb.x, xx (wb.y - 20), "B", .{ .0, .8, 1, 1 });

        Simp.set_shader_for_color(true);

        draw_point(wa, .{ 1., .4, 0, 1 }, 10.0);
        draw_point(wb, .{ .0, .8, 1, 1 }, 10.0);
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

my_font: *Simp.Dynamic_Font;

#import "Basic";
#import "Math";

Window  :: #import "Window_Creation";
Input   :: #import "Input";
Simp    :: #import "Simp";
Physics :: #import "Physics";