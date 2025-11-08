import numpy as np
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass

# this sucks, please do not use this... just a placeholder for bounding box data
@dataclass
class BoundingBox:
    x: float
    y: float
    width: float
    height: float
    confidence: float
    class_id: int
    class_name: str = ""
    
    @property
    def x_min(self) -> float:
        return self.x - self.width / 2
    
    @property
    def x_max(self) -> float:
        return self.x + self.width / 2
    
    @property
    def y_min(self) -> float:
        return self.y - self.height / 2
    
    @property
    def y_max(self) -> float:
        return self.y + self.height / 2
    
    @property
    def area(self) -> float:
        return self.width * self.height
    
    @property
    def center(self) -> Tuple[float, float]:
        return (self.x, self.y)

class FramingCostAnalyzer:
    
    def __init__(self, 
                 image_width: int = 640,
                 image_height: int = 480,
                 weights: Optional[Dict[str, float]] = None):
        self.image_width = image_width
        self.image_height = image_height
        self.aspect_ratio = image_width / image_height
        
        self.weights = weights or {
            'rule_of_thirds': 1.0,
            'subject_scale': 1.0,
            'frame_balance': 0.8,
            'headroom': 0.6,
            'leading_room': 0.6,
            'edge_distance': 0.5,
            'symmetry': 0.4,
        }
        
        self.thirds_points = [
            (1/3, 1/3), (2/3, 1/3),
            (1/3, 2/3), (2/3, 2/3)
        ]
    
    def rule_of_thirds_cost(self, bbox: BoundingBox) -> float:
        subject_center = np.array([bbox.x, bbox.y])
        
        min_distance = min(
            np.linalg.norm(subject_center - np.array(point))
            for point in self.thirds_points
        )
        
        diagonal = np.sqrt(2)
        cost = (min_distance / diagonal) ** 2
        
        return cost
    
    def subject_scale_cost(self, 
                          bbox: BoundingBox,
                          ideal_scale: float = 0.3,
                          tolerance: float = 0.1) -> float:
        subject_size = bbox.area
        
        deviation = abs(subject_size - ideal_scale)
        if deviation <= tolerance:
            return 0.0
        
        cost = ((deviation - tolerance) / ideal_scale) ** 2
        return cost
    
    def headroom_cost(self,
                     bbox: BoundingBox,
                     ideal_headroom: float = 0.15,
                     tolerance: float = 0.05) -> float:
        actual_headroom = 1.0 - bbox.y_max
        
        deviation = abs(actual_headroom - ideal_headroom)
        if deviation <= tolerance:
            return 0.0
        
        cost = ((deviation - tolerance) / ideal_headroom) ** 2
        
        if actual_headroom < 0:
            cost += 10.0
        
        return cost
    
    def leading_room_cost(self,
                         bbox: BoundingBox,
                         direction: str = 'right',
                         ideal_leading: float = 0.2) -> float:
        if direction == 'right':
            actual_leading = 1.0 - bbox.x_max
            trailing = bbox.x_min
        else:
            actual_leading = bbox.x_min
            trailing = 1.0 - bbox.x_max
        
        room_difference = trailing - actual_leading
        
        if room_difference > 0:
            cost = room_difference ** 2
        else:
            deviation = abs(actual_leading - ideal_leading)
            cost = (deviation / ideal_leading) ** 2
        
        return cost
    
    def frame_balance_cost(self, bboxes: List[BoundingBox]) -> float:
        if not bboxes:
            return 0.0
        
        total_weight = 0.0
        weighted_x = 0.0
        weighted_y = 0.0
        
        for bbox in bboxes:
            weight = bbox.area * bbox.confidence
            weighted_x += bbox.x * weight
            weighted_y += bbox.y * weight
            total_weight += weight
        
        if total_weight == 0:
            return 0.0
        
        center_of_mass_x = weighted_x / total_weight
        center_of_mass_y = weighted_y / total_weight
        
        frame_center = np.array([0.5, 0.5])
        com = np.array([center_of_mass_x, center_of_mass_y])
        
        distance = np.linalg.norm(com - frame_center)
        cost = (distance / 0.5) ** 2
        
        return cost
    
    def edge_distance_cost(self, bbox: BoundingBox, min_margin: float = 0.05) -> float:
        distances = [
            bbox.x_min,
            1.0 - bbox.x_max,
            bbox.y_min,
            1.0 - bbox.y_max
        ]
        
        min_distance = min(distances)
        
        if min_distance >= min_margin:
            return 0.0
        
        cost = ((min_margin - min_distance) / min_margin) ** 2
        
        return cost
    
    def symmetry_cost(self, bboxes: List[BoundingBox]) -> float:
        if len(bboxes) < 2:
            return 0.0
        
        left_weight = 0.0
        right_weight = 0.0
        
        for bbox in bboxes:
            weight = bbox.area * bbox.confidence
            if bbox.x < 0.5:
                left_weight += weight * (0.5 - bbox.x)
            else:
                right_weight += weight * (bbox.x - 0.5)
        
        total_weight = left_weight + right_weight
        if total_weight == 0:
            return 0.0
        
        imbalance = abs(left_weight - right_weight) / total_weight
        cost = imbalance ** 2
        
        return cost
    
    def multi_subject_relationship_cost(self, 
                                       bboxes: List[BoundingBox],
                                       ideal_separation: float = 0.15) -> float:
        if len(bboxes) < 2:
            return 0.0
        
        cost = 0.0
        num_pairs = 0
        
        for i in range(len(bboxes)):
            for j in range(i + 1, len(bboxes)):
                bbox1, bbox2 = bboxes[i], bboxes[j]
                
                center_dist = np.linalg.norm(
                    np.array(bbox1.center) - np.array(bbox2.center)
                )
                
                min_separation = (bbox1.width + bbox2.width) / 2
                
                if center_dist < min_separation:
                    cost += ((min_separation - center_dist) / min_separation) ** 2
                elif center_dist < ideal_separation:
                    cost += ((ideal_separation - center_dist) / ideal_separation) ** 2
                
                num_pairs += 1
        
        return cost / max(num_pairs, 1)
    
    def total_framing_cost(self,
                          bboxes: List[BoundingBox],
                          primary_subject_idx: int = 0,
                          subject_direction: str = 'right',
                          **kwargs) -> Dict[str, float]:
        if not bboxes:
            return {'total': float('inf'), 'individual_costs': {}}
        
        primary = bboxes[primary_subject_idx]
        
        costs = {
            'rule_of_thirds': self.rule_of_thirds_cost(primary),
            'subject_scale': self.subject_scale_cost(primary, **kwargs),
            'headroom': self.headroom_cost(primary, **kwargs),
            'leading_room': self.leading_room_cost(primary, subject_direction, **kwargs),
            'frame_balance': self.frame_balance_cost(bboxes),
            'edge_distance': self.edge_distance_cost(primary, **kwargs),
            'symmetry': self.symmetry_cost(bboxes),
        }
        
        total_cost = sum(
            costs[key] * self.weights.get(key, 1.0)
            for key in costs
        )
        
        return {
            'total': total_cost,
            'individual_costs': costs
        }
    
    def analyze_frame(self, 
                     bboxes: List[BoundingBox],
                     primary_subject_idx: int = 0,
                     verbose: bool = False) -> Dict:
        result = self.total_framing_cost(bboxes, primary_subject_idx)
        
        recommendations = []
        costs = result['individual_costs']
        
        if costs['rule_of_thirds'] > 0.1:
            recommendations.append("Move subject toward rule of thirds intersections")
        
        if costs['subject_scale'] > 0.1:
            primary = bboxes[primary_subject_idx]
            if primary.area < 0.3:
                recommendations.append("Move closer - subject too small")
            else:
                recommendations.append("Move back - subject too large")
        
        if costs['headroom'] > 0.1:
            recommendations.append("Adjust headroom (vertical positioning)")
        
        if costs['leading_room'] > 0.1:
            recommendations.append("Provide more leading room in direction of motion")
        
        if costs['edge_distance'] > 0.1:
            recommendations.append("Subject too close to frame edge")
        
        result['recommendations'] = recommendations
        result['quality_score'] = max(0, 100 - result['total'] * 50)
        
        if verbose:
            self._print_analysis(result, bboxes)
        
        return result
    
    def _print_analysis(self, result: Dict, bboxes: List[BoundingBox]):
        print("\n" + "="*50)
        print("FRAMING ANALYSIS")
        print("="*50)
        print(f"Quality Score: {result['quality_score']:.1f}/100")
        print(f"Total Cost: {result['total']:.3f}")
        print("\nIndividual Costs:")
        for name, cost in result['individual_costs'].items():
            print(f"  {name:20s}: {cost:.3f}")
        print("\nRecommendations:")
        for i, rec in enumerate(result['recommendations'], 1):
            print(f"  {i}. {rec}")
        print(f"\nDetected {len(bboxes)} object(s)")
        print("="*50 + "\n")


def parse_yolo_detection(detection: List) -> BoundingBox:
    return BoundingBox(
        x=detection[0],
        y=detection[1],
        width=detection[2],
        height=detection[3],
        confidence=detection[4],
        class_id=int(detection[5]),
        class_name=detection[6] if len(detection) > 6 else ""
    )


if __name__ == "__main__":
    yolo_detections = [
        [0.5, 0.4, 0.3, 0.5, 0.95, 0, "person"],
        [0.8, 0.7, 0.15, 0.2, 0.85, 64, "potted plant"]
    ]
    
    bboxes = [parse_yolo_detection(det) for det in yolo_detections]
    
    analyzer = FramingCostAnalyzer(image_width=1920, image_height=1080)
    
    result = analyzer.analyze_frame(bboxes, primary_subject_idx=0, verbose=True)
    
    print(f"\nOptimization target: Minimize total cost = {result['total']:.3f}")