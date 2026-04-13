import os
from typing import Optional

from launch import SomeSubstitutionsType, logging
from launch.utilities import perform_substitutions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix

from machinekit import rtapi, hal
from .hal_ordered_action import HalOrderedNode, HalThreadedReadyAction


class HalRTNode(HalOrderedNode, HalThreadedReadyAction):
    """Action that loads a HAL RT component ROS node."""

    def __init__(
        self,
        *,
        component: SomeSubstitutionsType,
        package: Optional[SomeSubstitutionsType] = None,
        hal_name=None,
        **kwargs,
    ) -> None:
        """
        Construct a HalRTNode action.

        Similar to :class:`launch.actions.Node`, this action launches a
        ROS node, but as a real time HAL component.
        """
        if package is not None:
            pkg_prefix = FindPackagePrefix(package=package)
            path = PathJoinSubstitution([pkg_prefix, "lib", component])
        else:
            path = component
        hal_name = hal_name or os.path.basename(component)

        # Pretend the component (a loadable plugin) is the full path
        # of an executable, thereby forced into the executable `Node`
        # scheme.  "Forced," for sure.
        super().__init__(
            hal_name=hal_name, package=None, executable=path, **kwargs
        )
        self.__package = package
        self.__component = component
        self.__component_path = path
        self.__logger = logging.get_logger(f"{__name__}({self.hal_name})")

    def is_ready(self, context):
        # If `loadrt()` call hasn't returned, not ready
        if not super().is_ready(context):
            return False
        # Check if component is registered and become ready
        if self.hal_name not in hal.components:
            self.__logger.debug(
                f"...HAL comp {self.hal_name} not yet registered"
            )
            return False
        elif hal.components[self.hal_name].state != hal.COMP_READY:
            self.__logger.debug("...HAL comp not yet ready")
            return False
        else:
            return True

    def _perform_substitutions(self, context):
        super()._perform_substitutions(context)
        # Node._perform_substitutions() may have appended
        # LocalSubstitution("ros_specific_arguments['ns']") to self.cmd when
        # namespace is set. execute_deferred_cb runs in a thread where
        # context.locals is unreliable (popped by the event loop before the
        # thread runs). Pre-resolve these LocalSubstitutions to concrete
        # TextSubstitutions now, while context is still valid.
        from launch.substitutions import LocalSubstitution, TextSubstitution
        ns = self.expanded_node_namespace
        ros_specific_arguments = {}
        if ns and ns != self.UNSPECIFIED_NODE_NAMESPACE:
            ros_specific_arguments['ns'] = f'__ns:={ns}'
        for i, sub_list in enumerate(self.cmd):
            new_sub_list = []
            changed = False
            for sub in sub_list:
                if isinstance(sub, LocalSubstitution):
                    try:
                        value = eval(  # noqa: S307
                            sub.expression,
                            {'ros_specific_arguments': ros_specific_arguments},
                        )
                        new_sub_list.append(TextSubstitution(text=value))
                        changed = True
                    except (KeyError, NameError, TypeError):
                        new_sub_list.append(sub)
                else:
                    new_sub_list.append(sub)
            if changed:
                self.cmd[i] = new_sub_list

    def execute_deferred_cb(self, context):
        # Expand command substitutions
        cmd = [perform_substitutions(context, c) for c in self.cmd]
        comp_path = cmd[0]
        self.__logger.info(f"Loading HAL RT component {comp_path}")
        self.comp_args = ",".join(cmd[1:])
        self.__logger.info(f"  args: ARGV={self.comp_args}")

        rtapi.loadrt(comp_path, ARGV=self.comp_args)

        self.__logger.info("loadrt complete")
