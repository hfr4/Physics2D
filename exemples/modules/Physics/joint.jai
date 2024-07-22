Joint :: struct {
    A, B: Anchor;
}

make_joint :: (A: *Collider, point_on_A: Vector2, B: *Collider, point_on_B: Vector2) -> Joint {
    return .{ make_anchor(A, point_on_A), make_anchor(B, point_on_B) };
}

get_positions :: (using joint: Joint) -> Vector2, Vector2 {
    return get_position(A), get_position(B);
}