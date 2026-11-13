"""pytest-only: resolve stair_supervisor from the catkin devel space first.

Several test modules prepend the package src directory to sys.path. In one pytest process the first
such import would bind the package without its generated messages, breaking
later `stair_supervisor.msg` imports. The devel package extends its path to src,
so tests still exercise the source tree. (catkin runs each file separately.)
"""
try:
    import stair_supervisor.msg  # noqa: F401
except ImportError:  # no devel space: plain library tests still run
    pass
