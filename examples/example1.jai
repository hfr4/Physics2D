#import "Basic";
#import "Math";

Window  :: #import "Window_Creation";
Input   :: #import "Input";
Simp    :: #import "Simp";

Physics :: #import "Physics";

window        : Window.Window_Type;
window_width  : s32 = 1100;
window_height : s32 = 900;

current_time: float64;
last\  _time: float64;

colliders: [..] *Physics.Collider;

yotsuba1, yotsuba2, yotsuba3, yotsuba4: Simp.Texture;

My_Collider_Userdata :: struct {
    texture: *Simp.Texture;
}

data1, data2, data3, data4: My_Collider_Userdata;

circle1, circle2:                               Physics.Collider;
rectangle1, rectangle2, rectangle3, rectangle4: Physics.Collider;
wall_l, wall_r, wall_t, wall_b:                 Physics.Collider;

main :: () {
    #if OS == .WINDOWS {
        Windows :: #import "Windows";
        Windows.SetProcessDPIAware();
        Windows.timeBeginPeriod(1);
    }

    window = Window.create_window(window_width, window_height, "Physics");

    Simp.set_render_target(window, .LEFT_HANDED);

    Simp.texture_load_from_file(*yotsuba1, "assets/yotsuba1.png");
    Simp.texture_load_from_file(*yotsuba2, "assets/yotsuba2.png");
    Simp.texture_load_from_file(*yotsuba3, "assets/yotsuba3.png");
    Simp.texture_load_from_file(*yotsuba4, "assets/yotsuba4.png");

    circle1    = Physics.make_circle(.{300, 300}, 50.0, 1.0, 0.5, 0.05);
    circle2    = Physics.make_circle(.{300, 600}, 50.0, 1.0, 0.5, 0.05);

    rectangle1 = Physics.make_rectangle(.{ 500, 200 }, xx yotsuba1.width, xx yotsuba1.height, 1.0, 0.5, 0.05);
    rectangle2 = Physics.make_rectangle(.{ 700, 300 }, xx yotsuba2.width, xx yotsuba2.height, 1.0, 0.5, 0.05);
    rectangle3 = Physics.make_rectangle(.{ 500, 700 }, xx yotsuba3.width, xx yotsuba3.height, 1.0, 0.5, 0.05);
    rectangle4 = Physics.make_rectangle(.{ 700, 700 }, xx yotsuba4.width, xx yotsuba4.height, 1.0, 0.5, 0.05);

    W := cast(float) window_width;
    H := cast(float) window_height;
    O := 50.0;

    wall_l     = Physics.make_rectangle(.{ W/2.0, -O    }, W      , O * 2.0, 0.0, 0.5, 0.05);
    wall_r     = Physics.make_rectangle(.{ W/2.0, H + O }, W      , O * 2.0, 0.0, 0.5, 0.05);
    wall_t     = Physics.make_rectangle(.{-O    , H/2.0 }, O * 2.0, H      , 0.0, 0.5, 0.05);
    wall_b     = Physics.make_rectangle(.{ W + O, H/2.0 }, O * 2.0, H      , 0.0, 0.5, 0.05);

    array_add(*colliders, *circle1, *rectangle1, *rectangle2, *rectangle3, *rectangle4, *circle2, *wall_l, *wall_r, *wall_t, *wall_b);

    data1.texture = *yotsuba1;
    data2.texture = *yotsuba2;
    data3.texture = *yotsuba3;
    data4.texture = *yotsuba4;

    rectangle1.userdata = *data1;
    rectangle2.userdata = *data2;
    rectangle3.userdata = *data3;
    rectangle4.userdata = *data4;

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

        if Input.input_button_states[Input.Key_Code.ESCAPE] & .START then quit = true;

        FORCE :: 1000.0;

        UP    :: Vector2.{  0, -1 };
        DOWN  :: Vector2.{  0,  1 };
        LEFT  :: Vector2.{ -1,  0 };
        RIGHT :: Vector2.{  1,  0 };

        if Input.input_button_states[Input.Key_Code.ARROW_UP]    & .DOWN Physics.add_force(*circle1, FORCE * UP);
        if Input.input_button_states[Input.Key_Code.ARROW_DOWN]  & .DOWN Physics.add_force(*circle1, FORCE * DOWN);
        if Input.input_button_states[Input.Key_Code.ARROW_LEFT]  & .DOWN Physics.add_force(*circle1, FORCE * LEFT);
        if Input.input_button_states[Input.Key_Code.ARROW_RIGHT] & .DOWN Physics.add_force(*circle1, FORCE * RIGHT);

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

    for i: 0..colliders.count - 2 {
        for j: i + 1..colliders.count - 1 {
            contact, ok := Physics.make_contact(colliders[i], colliders[j]);
            if ok Physics.solve(*contact);
        }
    }

    for colliders {

        Simp.set_shader_for_color(true);

        if it.kind == {
            case .CIRCLE;
                draw_circle(it.position, it.circle.radius, .{ 1, 1, 1, 1 }, 2.0);
                draw_segment(it.position, it.position + Vector2.{ cos(it.rotation), sin(it.rotation) } * it.circle.radius, .{ 1, 1, 1, 1 }, 2.0);

            case .POLYGON;
                vertices := Physics.get_vertices(it.polygon);
                draw_polygon(vertices, .{ 1, 1, 1, 1 }, 2.0);
                if it.userdata != null {
                    using my_userdata := cast(*My_Collider_Userdata) it.userdata;
                    Simp.set_shader_for_images(texture);
                    Simp.immediate_quad(vertices[0], vertices[1], vertices[2], vertices[3], .{ 1, 1, 1, 1 });
                }
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