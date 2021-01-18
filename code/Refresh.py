class Refresh():
    """
    Dispatches refresh events (F5) is created.
    """
    listeners = []

    @classmethod
    def addListener(self, listener):
        Refresh.listeners.append(listener)

    @classmethod
    def sendRefresh(self):
        for listener in Refresh.listeners:
            listener()
