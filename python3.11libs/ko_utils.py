from collections.abc import Callable

def oneFromList(list,
                chooser: Callable[[list], int] = lambda list: list[0],
                default=None,
                must_exist: bool = False,
                message_on_fail: str = "No valid items in list"):
    """
    Return one element from list.
    """
    if list:
        return chooser(list)
    else:
        if must_exist:
            raise Exception(message_on_fail)
        else:
            return default

def firstFromList(list, default=None, must_exist: bool = False, message_on_fail: str = "No valid items in list"):
    return oneFromList(list, lambda list: list[0], default, must_exist, message_on_fail)

def lastFromList(list, default=None, must_exist: bool = False, message_on_fail: str = "No valid items in list"):
    return oneFromList(list, lambda list: list[-1], default, must_exist, message_on_fail)
