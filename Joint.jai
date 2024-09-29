Anchor :: struct {
    collider:    *Collider;
    local_point: Vector2;
}

make_anchor :: (using collider: *Collider, world_point: Vector2) -> Anchor {
    return .{ collider, world_to_local(collider, world_point) };
}

get_position :: (using anchor: Anchor) -> Vector2 {
    return local_to_world(anchor.collider, anchor.local_point);
}

Joint :: struct {
    A, B: Anchor;
}

make_joint :: (A: *Collider, world_point_on_A: Vector2, B: *Collider, world_point_on_B: Vector2) -> Joint {
    return .{ make_anchor(A, world_point_on_A), make_anchor(B, world_point_on_B) };
}

get_positions :: (using joint: Joint) -> Vector2, Vector2 {
    return get_position(A), get_position(B);
}
