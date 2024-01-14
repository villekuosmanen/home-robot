import heapq


class BayesianFilter:
    def __init__(self):
        self.class_values = {}
        self.global_confidence = 0.45  # default

    def observe(self, class_id: int, confidence: float) -> bool:
        if class_id not in self.class_values:
            self.class_values[class_id] = ClassValues()

        required_confidence = self.calc_required_confidence(class_id)

        self.update_global_confidence()
        self.class_values[class_id].update_priors(confidence)
        return confidence >= required_confidence

    def calc_required_confidence(self, class_id: int) -> float:
        return (self.global_confidence + self.class_values[class_id].confidence()) / 2

    def update_global_confidence(self):
        # Logic to update global confidence
        total_confidence = sum(cv.confidence() for cv in self.class_values.values())
        total_classes = len(self.class_values)
        self.global_confidence = total_confidence / total_classes


class ClassValues:
    def __init__(self):
        self.top_obs = []  # length of 10, relpace with numpy if better

    def confidence(self) -> float:
        if not self.top_obs:
            return 0.45  # default value on no obs
        return sum(self.top_obs) / len(self.top_obs)

    def update_priors(self, confidence):
        if len(self.top_obs) < 10:
            heapq.heappush(self.top_obs, confidence)
        elif confidence > self.top_obs[0]:
            heapq.heappushpop(self.top_obs, confidence)
