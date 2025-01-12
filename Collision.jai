Collision_Key :: struct {
	A, B: *Body;
}

CollisionKeyHash :: (using key: Collision_Key) -> u32 {
	return (cast(u32) A * 31) ^ (cast(u32) B * 17);
}

ColisionKeyCompare :: (a: Collision_Key, b: Collision_Key) -> bool {
	return (a.A == b.A && a.B == b.B) || (a.A == b.B && a.B == b.A);
}

Collision :: struct {
	A, B:           *Body;
	R, I:           *Body; // Referent, Incident
	R_edge_index:   int;
	I_edge_index:   int;
	contacts_count: int;
	contacts:       [2] Contact;
}

Contact :: struct {
	position: Vector2; // On Referent
	normal:   Vector2; // From A to B
	depth:    float;   // Negative

	bias:                  float;
	N_combined_mass:       float;
	T_combined_mass:       float;
	N_accumulated_impulse: float;
	T_accumulated_impulse: float;
}

PreStep :: (using collision: *Collision, dt: float) {
	for 0..contacts_count - 1 {
		using contact := *contacts[it];

		pA := position - A.position;
		pB := position - B.position;

		N_combined_mass = 1.0 / (
			A.inv_mass + B.inv_mass +
			A.inv_inertia * (dot(pA, pA) - dot(pA, normal) * dot(pA, normal)) +
			B.inv_inertia * (dot(pB, pB) - dot(pB, normal) * dot(pB, normal))
		);

		tangent := Perpendicular(normal);

		T_combined_mass = 1.0 / (
			A.inv_mass + B.inv_mass +
			A.inv_inertia * (dot(pA, pA) - dot(pA, tangent) * dot(pA, tangent)) +
			B.inv_inertia * (dot(pB, pB) - dot(pB, tangent) * dot(pB, tangent))
		);

        ALLOWED_PENETRATION :: 0.01;
        BIAS_FACTOR         :: 0.20;

		separation := min(0.0, depth + ALLOWED_PENETRATION);
		bias = -BIAS_FACTOR * separation * (ifx dt > 0.0 then 1.0 / dt else 0.0);

		impulse := N_accumulated_impulse * normal + T_accumulated_impulse * tangent;

		ApplyImpulseAt(A, -impulse, pA);
		ApplyImpulseAt(B,  impulse, pB);
	}
}

Step :: (using collision: *Collision) {
	for 0..contacts_count - 1 {
		using contact := *contacts[it];

		pA := position - A.position;
		pB := position - B.position;

		tA := Perpendicular(pA);
		tB := Perpendicular(pB);

		{
			velocity := (B.linear_velocity + B.angular_velocity * tB) - (A.linear_velocity + A.angular_velocity * tA);

			new_accumulated_impulse := N_accumulated_impulse + N_combined_mass * (bias - dot(velocity, normal));
			new_accumulated_impulse = max(new_accumulated_impulse, 0.0);

			impulse := (new_accumulated_impulse - N_accumulated_impulse) * normal;

			ApplyImpulseAt(A, -impulse, pA);
			ApplyImpulseAt(B,  impulse, pB);

			N_accumulated_impulse = new_accumulated_impulse;
		}

		{
			velocity := (B.linear_velocity + B.angular_velocity * tB) - (A.linear_velocity + A.angular_velocity * tA);
			tangent  := Perpendicular(normal);
			friction := sqrt(A.friction * B.friction);

			new_accumulated_impulse := T_accumulated_impulse + T_combined_mass * -dot(velocity, tangent);
			new_accumulated_impulse = clamp(new_accumulated_impulse, N_accumulated_impulse * -friction, N_accumulated_impulse * friction);

			impulse := (new_accumulated_impulse - T_accumulated_impulse) * tangent;

			ApplyImpulseAt(A, -impulse, pA);
			ApplyImpulseAt(B,  impulse, pB);

			T_accumulated_impulse = new_accumulated_impulse;
		}
	}
}

Merge :: (target: *Collision, source: Collision) {
    merged := source;

	if target.R_edge_index == source.R_edge_index && target.I_edge_index == source.I_edge_index {
		for i: 0..source.contacts_count - 1 {
			for j: 0..target.contacts_count - 1 {
				merged.contacts[i].N_accumulated_impulse = target.contacts[j].N_accumulated_impulse;
				merged.contacts[i].T_accumulated_impulse = target.contacts[j].T_accumulated_impulse;
				break j;
			}
		}
	}

	target.* = merged;
}

Collide :: (A: *Body, B: *Body) -> Collision {
    collision: Collision;

	if      A.kind == .POLYGON && B.kind == .POLYGON collision = PolygonPolygon(A, B);
	else if A.kind == .CIRCLE  && B.kind == .CIRCLE  collision = CircleCircle(A, B);
	else if A.kind == .POLYGON && B.kind == .CIRCLE  collision = PolygonCircle(A, B);
	else if A.kind == .CIRCLE  && B.kind == .POLYGON collision = PolygonCircle(B, A);

	return collision;
}

CircleCircle :: (A: *Body, B: *Body) -> Collision {
    collision: Collision;

    if distance(A.position, B.position) > A.circle.radius + B.circle.radius return collision;

    collision.A            = A;
    collision.B            = B;
	collision.R            = A;
	collision.I            = B;
	collision.R_edge_index = 1;
	collision.I_edge_index = 1;

	normal        := normalize(B.position - A.position);
    position_on_A := A.position + normal * A.circle.radius;
    position_on_B := B.position - normal * B.circle.radius;

	contact := *collision.contacts[0];
	contact.position = position_on_A;
	contact.normal   = normal;
	contact.depth    = -distance(position_on_A, position_on_B);

	collision.contacts_count = 1;

    return collision;
}

PolygonCircle :: (A: *Body, B: *Body) -> Collision {
    collision: Collision;

	A_vertices_data := PolygonWorldVertices(A);
	A_vertices      := array_view(A_vertices_data, 0, A.polygon.vertices_count);

	edge, edge_index := ClosestEdgeFromPoint(A_vertices, B.position);
    point            := ClosestPointOnEdge(edge, B.position);
    dist             := distance(point, B.position);
    is_inside        := IsPointInsidePolygon(A_vertices, B.position);

    if !is_inside && dist > B.circle.radius return collision;

	collision.A            = A;
    collision.B            = B;
	collision.R            = A;
	collision.I            = B;
	collision.R_edge_index = edge_index;
	collision.I_edge_index = 1;

	normal0 := normalize(point - B.position);
    normal1 := ifx is_inside then normal0 else -normal0;

    position_on_A := B.position + normal0 * dist;
    position_on_B := B.position - normal1 * B.circle.radius;

	contact := *collision.contacts[0];
	contact.position = position_on_A;
	contact.normal   = normal1;
	contact.depth    = -distance(position_on_A, position_on_B);

	collision.contacts_count = 1;

    return collision;
}

PolygonPolygon :: (A: *Body, B: *Body) -> Collision {
	collision: Collision;

	A_vertices_data := PolygonWorldVertices(A);
	A_vertices      := array_view(A_vertices_data, 0, A.polygon.vertices_count);

	B_vertices_data := PolygonWorldVertices(B);
	B_vertices      := array_view(B_vertices_data, 0, B.polygon.vertices_count);

	R_edge, R_edge_index, A_is_referent, is_colliding := ComputeReferentEdge(A_vertices, B_vertices);

	if !is_colliding return collision;

	I_vertices    := ifx A_is_referent then B_vertices else A_vertices;
	R_edge_normal := Perpendicular(normalize(R_edge[0] - R_edge[1]));

	I_edge, I_edge_index := ComputeIncidentEdge(I_vertices, R_edge_normal);

	if ClipEdgeWithLine(*I_edge, R_edge[0],  R_edge_normal) < 2 return collision;
	if ClipEdgeWithLine(*I_edge, R_edge[1], -R_edge_normal) < 2 return collision;

	collision.A            = A;
	collision.B            = B;
	collision.R            = ifx A_is_referent then A else B;
	collision.I            = ifx A_is_referent then B else A;
	collision.R_edge_index = R_edge_index;
	collision.I_edge_index = I_edge_index;

	for 0..1 {
		depth := dot(I_edge[it] - R_edge[0], R_edge_normal);
		if depth < 0.0 {
			contact := *collision.contacts[collision.contacts_count];
			contact.position = I_edge[it] - R_edge_normal * depth;
			contact.normal   = ifx A_is_referent then R_edge_normal else -R_edge_normal;
			contact.depth    = depth;

			collision.contacts_count += 1;
		}
	}

	return collision;
}
