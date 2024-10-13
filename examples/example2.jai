colliders: [..] *Physics.Collider;
mouse_anchor: Physics.Anchor;

polygon, rectangle, circle: Physics.Collider;
walls: [4] Physics.Collider;

colliders_joint: Physics.Joint;

main :: () {
    #if OS == .WINDOWS {
        Windows :: #import "Windows";
        Windows.SetProcessDPIAware();
        Windows.timeBeginPeriod(1);
    }

    window = Window.create_window(window_width, window_height, "Physics");

    Simp.set_render_target(window, .LEFT_HANDED);

    polygon   = Physics.make_polygon(.[ .{300, 300}, .{400, 300}, .{300, 400} ], 1.0, 0.5, 0.05);
    rectangle = Physics.make_rectangle(.{500, 200}, 200, 50 , 1.0, 0.5, 0.05);
    circle    = Physics.make_circle(.{700, 300}, 50.0, 1.0, 0.5, 0.05);

    // colliders_joint = Physics.make_joint(*polygon, polygon.position, *circle, circle.position);

    W := cast(float) window_width;
    H := cast(float) window_height;
    C := Vector2.{ W / 2.0, H / 2.0 };
    O := 50.0;

    walls[0] = Physics.make_rectangle(.{ C.x  , -O    }, W      , O * 2.0, 0.0, 0.5, 0.05);
    walls[1] = Physics.make_rectangle(.{ C.x  , H + O }, W      , O * 2.0, 0.0, 0.5, 0.05);
    walls[2] = Physics.make_rectangle(.{-O    , C.y   }, O * 2.0, H      , 0.0, 0.5, 0.05);
    walls[3] = Physics.make_rectangle(.{ W + O, C.y   }, O * 2.0, H      , 0.0, 0.5, 0.05);

    array_add(*colliders, *polygon, *rectangle, *circle, *walls[0], *walls[1], *walls[2], *walls[3]);

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

                for colliders {
                    if Physics.is_point_inside(it, mouse_position) {
                        mouse_anchor = Physics.make_anchor(it, mouse_position);
                    }
                }
            }

            if key == .MOUSE_BUTTON_LEFT && released {
                mouse_anchor.collider = null;
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

    if mouse_anchor.collider {
        anchor_position := Physics.get_position(mouse_anchor);

        if distance(mouse_position, anchor_position) > 200.0 {
            mouse_force := (mouse_position - anchor_position) * 5.0;
            Physics.add_force_at_point(mouse_anchor.collider, mouse_force, anchor_position);
        }
    }
/*
    {
        a_anchor_position := Physics.get_position(colliders_joint.A);
        b_anchor_position := Physics.get_position(colliders_joint.B);

        force: Vector2;

        dist := distance(a_anchor_position, b_anchor_position);

        if dist > 300.0 {
            force = (a_anchor_position - b_anchor_position) * 2.5;
        } else if dist < 100.0 {
            force = (b_anchor_position - a_anchor_position) * 2.5;
        }

        Physics.add_force_at_point(colliders_joint.A.collider, -force, a_anchor_position);
        Physics.add_force_at_point(colliders_joint.B.collider,  force, b_anchor_position);
    }
*/
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
            contact, ok := Physics.make_contact(colliders[i], colliders[j]);
            if ok Physics.solve(*contact);
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

    if mouse_anchor.collider {
        anchor_position := Physics.get_position(mouse_anchor);
        draw_segment(anchor_position, mouse_position, .{ 1, 0, 0, 1}, 2.0);
        draw_circle(anchor_position, 5.0, .{ 1, 0, 0, 1}, 2.0);
    }
/*
    {
        a_position, b_position := Physics.get_positions(colliders_joint);
        draw_segment(a_position, b_position, .{ 0, 1, 1, 1}, 2.0);
        draw_circle(a_position, 5.0, .{ 0, 1, 1, 1}, 2.0);
        draw_circle(b_position, 5.0, .{ 0, 1, 1, 1}, 2.0);
    }
*/
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