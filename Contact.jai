Contact :: struct {
    A, B:     *Collider;
    anchor_A: Anchor;
    anchor_B: Anchor;
    normal:   Vector2;
    depth:    float;
}

make_contact :: (A: *Collider, B: *Collider) -> Contact, bool {
    contact: Contact;
    ok: bool;

    if      A.kind == .CIRCLE  && B.kind == .CIRCLE  contact, ok = circle_circle(A, B);
    else if A.kind == .POLYGON && B.kind == .CIRCLE  contact, ok = polygon_circle(A, B);
    else if A.kind == .CIRCLE  && B.kind == .POLYGON contact, ok = polygon_circle(B, A);
    else if A.kind == .POLYGON && B.kind == .POLYGON contact, ok = polygon_polygon(A, B);

    return contact, ok;
}

circle_circle :: (A: *Collider, B: *Collider) -> Contact, bool {
    contact: Contact;

    is_colliding := distance(A.position, B.position) <= A.circle.radius + B.circle.radius;
    if !is_colliding return contact, false;

    contact_normal   := normalize(B.position - A.position);
    world_point_on_A := B.position - contact_normal * B.circle.radius;
    world_point_on_B := A.position + contact_normal * A.circle.radius;

    contact.A        = A;
    contact.B        = B;
    contact.anchor_A = make_anchor(A, world_point_on_A);
    contact.anchor_B = make_anchor(B, world_point_on_B);
    contact.normal   = contact_normal;
    contact.depth    = distance(world_point_on_A, world_point_on_B);

    return contact, true;
}

polygon_circle :: (A: *Collider, B: *Collider) -> Contact, bool {
    contact: Contact;

    vertices  := get_vertices(A.polygon);
    closest   := find_closest_point_on_vertices(vertices, B.position);
    dist      := distance(closest, B.position);
    is_inside := is_point_inside_polygon(vertices, B.position);

    is_colliding := is_inside || dist <= B.circle.radius;
    if !is_colliding return contact, false;

    closest_to_B_normal := normalize(closest - B.position);
    contact_normal      := ifx is_inside then closest_to_B_normal else -closest_to_B_normal;
    world_point_on_A    := B.position - contact_normal * B.circle.radius;
    world_point_on_B    := B.position + closest_to_B_normal * dist;

    contact.A        = A;
    contact.B        = B;
    contact.anchor_A = make_anchor(A, world_point_on_A);
    contact.anchor_B = make_anchor(B, world_point_on_B);
    contact.normal   = contact_normal;
    contact.depth    = distance(world_point_on_A, world_point_on_B);

    return contact, true;
}

polygon_polygon :: (A: *Collider, B: *Collider) -> Contact, bool {
    contact: Contact;

    vertices_a := get_vertices(A.polygon);
    vertices_b := get_vertices(B.polygon);

    ab, is_colliding_ab := find_sat_collision(vertices_a, vertices_b);
    if !is_colliding_ab return contact, false;

    ba, is_colliding_ba := find_sat_collision(vertices_b, vertices_a);
    if !is_colliding_ba return contact, false;

    contact_normal   := ifx ab.depth < ba.depth then ab.normal else -ba.normal;
    world_point_on_A := ifx ab.depth < ba.depth then ab.point  else ba.point + ba.normal * ba.depth;
    world_point_on_B := ifx ab.depth > ba.depth then ba.point  else ab.point + ab.normal * ab.depth;

    contact.A        = A;
    contact.B        = B;
    contact.anchor_A = make_anchor(A, world_point_on_A);
    contact.anchor_B = make_anchor(B, world_point_on_B);
    contact.normal   = contact_normal;
    contact.depth    = distance(world_point_on_A, world_point_on_B);

    return contact, true;
}

solve :: (using contact: *Contact) {
    if is_static(A) && is_static(B) return;

    pa := get_position(anchor_A) - A.position;
    pb := get_position(anchor_B) - B.position;

    va := A.linear_velocity + A.angular_velocity * perpendicular(pa);
    vb := B.linear_velocity + B.angular_velocity * perpendicular(pb);
    V  := vb - va; // Relative velocity

    N   := normal;
    dvn := dot(V, N);

    colliders_are_moving_apart := dvn > 0.0;
    if colliders_are_moving_apart return;

    T   := N * dvn - V; // Tangent
    dvt := dot(V, T);

    E := min(A.elasticity, B.elasticity);
    F := min(A.friction, B.friction);
    M := A.inv_mass + B.inv_mass;

    can := cross(pa, N);
    cbn := cross(pb, N);
    cat := cross(pa, T);
    cbt := cross(pb, T);

    normal_magnitude  := -(1.0 + E) * dvn     / (M + A.inv_inertia * can * can + B.inv_inertia * cbn * cbn);
    tangent_magnitude := -(1.0 + E) * dvt * F / (M + A.inv_inertia * cat * cat + B.inv_inertia * cbt * cbt);

    impulse := N * normal_magnitude + T * tangent_magnitude;
    apply_impulse_at_point(A, -impulse, pa);
    apply_impulse_at_point(B,  impulse, pb);

    DEPTH_ALLOWANCE  :: 0.05;
    DEPTH_CORRECTION :: 0.45;

    correction := max(depth - DEPTH_ALLOWANCE, 0.0) / M * DEPTH_CORRECTION;
    move(A, correction * -N * A.inv_mass);
    move(B, correction *  N * B.inv_mass);
}
