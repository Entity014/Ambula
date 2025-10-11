def side_sign_y(side: str) -> float:
    # ซ้าย = +Y, ขวา = -Y (REP-103)
    return -1.0 if side.lower().startswith("left") else +1.0
