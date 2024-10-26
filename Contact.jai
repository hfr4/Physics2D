Contact :: struct {
    A, B:     *Collider;
    anchor_A: Anchor;
    anchor_B: Anchor;
    normal:   Vector2;
    depth:    float;
}

generate_contacts :: (A: *Collider, B: *Collider) -> [2] Contact, int {
    contacts: [2] Contact;
    count: int;

    if A.kind == .CIRCLE  && B.kind == .CIRCLE {
        contact, ok := circle_circle(A, B);
        if ok {
            contacts[0] = contact;
            count = 1;
        }
    }
    else if A.kind == .POLYGON && B.kind == .CIRCLE {
        contact, ok := polygon_circle(A, B);
        if ok {
            contacts[0] = contact;
            count = 1;
        }
    }
    else if A.kind == .CIRCLE  && B.kind == .POLYGON {
        contact, ok := polygon_circle(B, A);
        if ok {
            contacts[0] = contact;
            count = 1;
        }
    }
    else if A.kind == .POLYGON && B.kind == .POLYGON {
        contacts, count = polygon_polygon(A, B);
    }

    return contacts, count;
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

polygon_polygon :: (A: *Collider, B: *Collider) -> [2] Contact, int {
    contacts: [2] Contact;
    contacts_count: int;

    a_vertices := get_vertices(A.polygon);
    b_vertices := get_vertices(B.polygon);

    a_edge, a_depth := find_smallest_separating_edge_and_depth_using_SAT(a_vertices, b_vertices);
    if a_depth >= 0.0 return contacts, contacts_count;

    b_edge, b_depth := find_smallest_separating_edge_and_depth_using_SAT(b_vertices, a_vertices);
    if b_depth >= 0.0 return contacts, contacts_count;

    reference_is_A := a_depth > b_depth;

    ref_vertices    := ifx reference_is_A then a_vertices else b_vertices;
    inc_vertices    := ifx reference_is_A then b_vertices else a_vertices;
    ref_edge        := ifx reference_is_A then a_edge     else b_edge;
    ref_edge_normal := segment_normal(ref_edge);
    inc_edge        := find_edge_pointed_by_direction(inc_vertices, ref_edge_normal);

    clip, clip_count := clip_segment_with_polygon(inc_edge, ref_vertices);
    if !clip_count return contacts, contacts_count;

    for 0..clip_count - 1 {
        depth := dot(ref_edge[0] - clip[it], ref_edge_normal);

        EPSILON :: 0.05;
        if depth < EPSILON continue;

        world_point_on_ref := clip[it];
        world_point_on_inc := clip[it] + ref_edge_normal * depth;
        world_point_on_A   := ifx reference_is_A then world_point_on_ref else world_point_on_inc;
        world_point_on_B   := ifx reference_is_A then world_point_on_inc else world_point_on_ref;
        contact_normal     := ifx reference_is_A then ref_edge_normal    else -ref_edge_normal;

        contact: Contact;
        contact.A        = A;
        contact.B        = B;
        contact.anchor_A = make_anchor(A, world_point_on_A);
        contact.anchor_B = make_anchor(B, world_point_on_B);
        contact.normal   = contact_normal;
        contact.depth    = depth;

        contacts[contacts_count] = contact;
        contacts_count += 1;
    }

    return contacts, contacts_count;
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
