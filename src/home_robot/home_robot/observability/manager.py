from proto.action_pb2 import Action


class ObservabilityManager:
    def __init__(self):
        # Initialize if needed
        pass

    def process_action(self, name, sequence):
        # Create an Action proto object
        action = Action(name=name, sequence=sequence)

        # For now, print the Action proto
        # Later, this can be extended to publish the proto
        print(action)
