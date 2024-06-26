#
# Generated by erpcgen 1.9.0 on Tue May 31 17:37:31 2022.
#
# AUTOGENERATED - DO NOT EDIT
#

import erpc
from . import common, interface

# Client for BBCarService
class BBCarServiceService(erpc.server.Service):
    def __init__(self, handler):
        super(BBCarServiceService, self).__init__(interface.IBBCarService.SERVICE_ID)
        self._handler = handler
        self._methods = {
                interface.IBBCarService.RETURNDISTANCE_ID: self._handle_returndistance,
                interface.IBBCarService.RETURNSPEED_ID: self._handle_returnspeed,
                interface.IBBCarService.RETURNSTATUS_ID: self._handle_returnstatus,
            }

    def _handle_returndistance(self, sequence, codec):
        # Create reference objects to pass into handler for out/inout parameters.
        result = erpc.Reference()

        # Read incoming parameters.

        # Invoke user implementation of remote function.
        self._handler.returndistance(result)

        # Prepare codec for reply message.
        codec.reset()

        # Construct reply message.
        codec.start_write_message(erpc.codec.MessageInfo(
            type=erpc.codec.MessageType.kReplyMessage,
            service=interface.IBBCarService.SERVICE_ID,
            request=interface.IBBCarService.RETURNDISTANCE_ID,
            sequence=sequence))
        if result.value is None:
            raise ValueError("result is None")
        codec.write_float(result.value)

    def _handle_returnspeed(self, sequence, codec):
        # Create reference objects to pass into handler for out/inout parameters.
        result = erpc.Reference()

        # Read incoming parameters.

        # Invoke user implementation of remote function.
        self._handler.returnspeed(result)

        # Prepare codec for reply message.
        codec.reset()

        # Construct reply message.
        codec.start_write_message(erpc.codec.MessageInfo(
            type=erpc.codec.MessageType.kReplyMessage,
            service=interface.IBBCarService.SERVICE_ID,
            request=interface.IBBCarService.RETURNSPEED_ID,
            sequence=sequence))
        if result.value is None:
            raise ValueError("result is None")
        codec.write_float(result.value)

    def _handle_returnstatus(self, sequence, codec):
        # Create reference objects to pass into handler for out/inout parameters.
        result = erpc.Reference()

        # Read incoming parameters.

        # Invoke user implementation of remote function.
        self._handler.returnstatus(result)

        # Prepare codec for reply message.
        codec.reset()

        # Construct reply message.
        codec.start_write_message(erpc.codec.MessageInfo(
            type=erpc.codec.MessageType.kReplyMessage,
            service=interface.IBBCarService.SERVICE_ID,
            request=interface.IBBCarService.RETURNSTATUS_ID,
            sequence=sequence))
        if result.value is None:
            raise ValueError("result is None")
        codec.write_float(result.value)


