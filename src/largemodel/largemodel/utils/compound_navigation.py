COMPOUND_DESTINATIONS = (
    "a",
    "b",
    "c",
    "d",
    "沙发",
    "桌子",
    "椅子",
    "箱子",
    "办公室",
    "电梯",
    "大厅",
    "前台",
    "会议室",
    "充电桩",
)

# 只有会议室离开时必须先经过自身入口；其他位置不添加来源入口。
EXIT_VIA_GATEWAY_DESTINATIONS = frozenset(("会议室",))

_NUMBERED_GATEWAYS = {
    "a": ("一号点入口", "a入口"),
    "b": ("二号点入口", "b入口"),
    "c": ("三号点入口", "c入口"),
    "d": ("四号点入口", "d入口"),
}


def gateway_names(destination):
    destination = str(destination).casefold()
    if destination in _NUMBERED_GATEWAYS:
        return _NUMBERED_GATEWAYS[destination]
    if destination == "会议室":
        return ("会议室门口", "会议室入口")
    return (f"{destination}入口", f"{destination}门口")


GATEWAY_POINT_NAMES = frozenset(
    gateway
    for destination in COMPOUND_DESTINATIONS
    for gateway in gateway_names(destination)
)


def find_symbol_by_name(target_points, point_name):
    normalized_name = str(point_name).casefold()
    for symbol, point in target_points.items():
        if not isinstance(point, dict):
            continue
        name = str(point.get("name", "")).strip(" ‘ ’ “ ” \" '").casefold()
        if name == normalized_name:
            return str(symbol)
    return None


def find_gateway_symbol(target_points, destination):
    for gateway_name in gateway_names(destination):
        symbol = find_symbol_by_name(target_points, gateway_name)
        if symbol is not None:
            return symbol
    return None


def compose_compound_route(
    target_points,
    target_symbol,
    source_symbol=None,
):
    target_symbol = str(target_symbol)
    target = target_points.get(target_symbol)
    if not isinstance(target, dict):
        return None

    target_name = str(target.get("name", "")).strip(" ‘ ’ “ ” \" '").casefold()
    target_gateway = find_gateway_symbol(target_points, target_name)
    if target_gateway is None:
        return None

    source_gateway = None
    if source_symbol is not None and str(source_symbol) != target_symbol:
        source = target_points.get(str(source_symbol))
        if isinstance(source, dict):
            source_name = str(source.get("name", "")).strip(
                " ‘ ’ “ ” \" '"
            ).casefold()
            if source_name in EXIT_VIA_GATEWAY_DESTINATIONS:
                source_gateway = find_gateway_symbol(target_points, source_name)

    route = [source_gateway, target_gateway, target_symbol]
    return [
        symbol
        for index, symbol in enumerate(route)
        if symbol is not None and (index == 0 or symbol != route[index - 1])
    ]
