from Refresh import Refresh

import cefpanda

class WebUI():
  """
  This is mainly a singleton to get the CEFPanda ui

  CEF is the Chromium Embedded Framework and CEFPanda is a tool that enables it's use
  with Python.
  """

  ui = None

  @classmethod
  def get(self):
    """
    Get UI

    Gets the UI singleton and initializes it if not already done.
    """
    if WebUI.ui is None:
      WebUI.ui = cefpanda.CEFPanda(
        transparent=True,
        size=[-1.0, 1.0, -1.0, 1.0]
      )
      Refresh.addListener(WebUI.load)
      WebUI.load()

    return WebUI.ui

  @classmethod
  def load(self):
    self.ui.load_file('ui/main.html')
