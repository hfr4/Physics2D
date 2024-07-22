Anchor :: struct {
    collider: *Collider;
    point:    Vector2;
}

make_anchor :: (using collider: *Collider, point: Vector2) -> Anchor {
    return .{ collider, world_to_local(collider, point) };
}

get_position :: (using anchor: Anchor) -> Vector2 {
    return local_to_world(anchor.collider, anchor.point);
}
