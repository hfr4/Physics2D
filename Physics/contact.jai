Contact :: struct {
    A, B:       *Collider;
    normal:     Vector2;
    depth:      float;
    point_on_A: Vector2;
    point_on_B: Vector2;
}

make_contact :: (A: *Collider, B: *Collider) -> Contact, bool {
    contact: Contact;
    ok: bool;

    if      A.kind == .CIRCLE  && B.kind == .CIRCLE  contact, ok = circle_circle_contact(A, B);
    else if A.kind == .POLYGON && B.kind == .CIRCLE  contact, ok = polygon_circle_contact(A, B);
    else if A.kind == .CIRCLE  && B.kind == .POLYGON contact, ok = polygon_circle_contact(B, A);
    else if A.kind == .POLYGON && B.kind == .POLYGON contact, ok = polygon_polygon_contact(A, B);

    return contact, ok;
}

circle_circle_contact :: (A: *Collider, B: *Collider) -> Contact, bool {
    contact: Contact;

    is_colliding := distance(A.position, B.position) <= A.circle.radius + B.circle.radius;
    if !is_colliding return contact, false;

    contact.A          = A;
    contact.B          = B;
    contact.normal     = normalize(B.position - A.position);
    contact.point_on_A = B.position - contact.normal * B.circle.radius;
    contact.point_on_B = A.position + contact.normal * A.circle.radius;
    contact.depth      = distance(contact.point_on_A, contact.point_on_B);

    return contact, true;
}

polygon_circle_contact :: (A: *Collider, B: *Collider) -> Contact, bool {
    contact: Contact;

    vertices  := get_vertices(A.polygon);
    point     := find_closest_point_on_vertices(vertices, B.position);
    dist      := distance(point, B.position);
    is_inside := is_point_inside_polygon(vertices, B.position);

    is_colliding := is_inside || dist <= B.circle.radius;
    if !is_colliding return contact, false;

    contact.A          = A;
    contact.B          = B;
    contact.normal     = ifx is_inside then normalize(point - B.position) else normalize(B.position - point);
    contact.point_on_A = B.position - contact.normal * B.circle.radius;
    contact.point_on_B = B.position - contact.normal * dist;
    contact.depth      = distance(contact.point_on_A, contact.point_on_B);

    return contact, true;
}

polygon_polygon_contact :: (A: *Collider, B: *Collider) -> Contact, bool {
    contact: Contact;

    vertices_a := get_vertices(A.polygon);
    vertices_b := get_vertices(B.polygon);

    is_colliding: bool;

    ab:, is_colliding = find_sat_collision(vertices_a, vertices_b);
    if !is_colliding return contact, false;

    ba:, is_colliding = find_sat_collision(vertices_b, vertices_a);
    if !is_colliding return contact, false;

    contact.A          = A;
    contact.B          = B;
    contact.normal     = ifx ab.depth < ba.depth then ab.normal else -ba.normal;
    contact.point_on_A = ifx ab.depth < ba.depth then ab.point  else  ba.point + ba.normal * ba.depth;
    contact.point_on_B = ifx ab.depth > ba.depth then ba.point  else  ab.point + ab.normal * ab.depth;
    contact.depth      = distance(contact.point_on_A, contact.point_on_B);

    return contact, true;
}

solve :: (using contact: *Contact) {
    if is_static(A) && is_static(B) return;

    a := point_on_A - A.position;
    b := point_on_B - B.position;

    va := A.linear_velocity + A.angular_velocity * perpendicular(a);
    vb := B.linear_velocity + B.angular_velocity * perpendicular(b);
    v  := vb - va; // Relative velocity

    n  := normal;
    vn := dot(v, n);

    // Don't solve contact because the two colliders are moving away from each other.
    if vn > 0.0 return; 

    t  := n * vn - v;
    vt := dot(v, t);

    e := min(A.restitution, B.restitution); // Coefficient of elasticity  
    f := min(A.friction, B.friction);       // Coefficient of friction

    normal_magnitude  := -(1.0 + e) * vn     / (A.inv_mass + B.inv_mass + A.inv_inertia * cross(a, n) * cross(a, n) + B.inv_inertia * cross(b, n) * cross(b, n));
    tangent_magnitude := -(1.0 + e) * vt * f / (A.inv_mass + B.inv_mass + A.inv_inertia * cross(a, t) * cross(a, t) + B.inv_inertia * cross(b, t) * cross(b, t));

    impulse := n * normal_magnitude + t * tangent_magnitude;
    apply_impulse_at_point(A, -impulse, a);
    apply_impulse_at_point(B,  impulse, b);

    // Apply position correction

    POSITION_CORRECTION :: 0.9;

    correction_amount := depth / (A.inv_mass + B.inv_mass) * POSITION_CORRECTION;
    correction_vector := n * correction_amount;
    move(A, -correction_vector * A.inv_mass);
    move(B,  correction_vector * B.inv_mass);
}