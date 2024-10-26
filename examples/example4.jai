colliders: [..] *Physics.Collider;

borders: [4] Physics.Collider;
rects: [..] Physics.Collider;

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

    borders[1] = Physics.make_rectangle(.{ C.x  , H + O - 200 }, 800      , 100, 0.0, 0.5, 0.05);
    // Physics.rotate(*borders[1], 0.001);
    array_add(*colliders, *borders[1]);

    quit := false;
    while !quit {
        current_time = seconds_since_init();
        dt := cast(float)(current_time - last_time);
        Clamp(*dt, 0, 0.1);
        last_time = current_time;

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

        for Input.input_button_states {
            key      := cast(Input.Key_Code) it_index;
            pressed  := it & .START;
            released := it & .END;
            down     := it & .DOWN;

            if key == .ESCAPE && pressed then quit = true;

            if key == .MOUSE_BUTTON_LEFT && pressed {
                mx, my         := Window.get_mouse_pointer_position(right_handed = false);
                mouse_position := Vector2.{ cast(float) mx , cast(float) my };
                rect := New(Physics.Collider);
                rect.* = Physics.make_rectangle(mouse_position, 200, 20, 1.0, 0.5, 0.05);

                array_add(*colliders, rect);
            }

            if key == .MOUSE_BUTTON_LEFT && released {
            }
        }

        draw_one_frame(dt);

        reset_temporary_storage();
        sleep_milliseconds(10);
    }
}

draw_one_frame :: (dt: float) {
    Simp.clear_render_target(.1, .1, .1, 1);

    Simp.set_shader_for_color(true);

    mx, my         := Window.get_mouse_pointer_position(right_handed = false);
    mouse_position := Vector2.{ cast(float) mx , cast(float) my };

    for colliders {
        GRAVITY :: Vector2.{ 0, 250 };

        gravity_force := GRAVITY * it.mass;
        Physics.add_force(it, gravity_force);
    }

    for colliders {
        Physics.update(it, dt);
    }

    for i: 0..colliders.count - 2 {
        for j: i + 1..colliders.count - 1 {
            contacts, count := Physics.generate_contacts(colliders[i], colliders[j]);
            for 0..count - 1 Physics.solve(*contacts[it]);
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
    for vertex: vertices {
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