# Vision-Language Integration: Hands-On Exercises

## Exercise Overview

This exercise provides hands-on experience with implementing vision-language integration systems in Vision-Language-Action (VLA) systems. Students will build and test a complete vision-language pipeline that connects linguistic references with visual entities.

## Exercise Objectives

By completing this exercise, students will:

1. Implement a vision-language grounding system
2. Integrate visual and linguistic processing pipelines
3. Develop object identification and scene understanding capabilities
4. Test and validate the vision-language integration performance

## Exercise 1: Basic Vision-Language Setup

### Task 1.1: Set up Vision-Language Environment

Create a Python environment with vision-language integration:

```python
import torch
import clip

class VisionLanguageSystem:
    def __init__(self):
        """
        Initialize vision-language system with CLIP model
        """
        self.clip_model, self.preprocess = clip.load("ViT-B/32")
        self.clip_model.eval()
        print("Vision-Language system initialized with CLIP model")

    def compute_similarity(self, image, text):
        """
        Compute similarity between image and text
        """
        # Preprocess image
        image_input = self.preprocess(image).unsqueeze(0)

        # Tokenize text
        text_input = clip.tokenize([text])

        with torch.no_grad():
            image_features = self.clip_model.encode_image(image_input)
            text_features = self.clip_model.encode_text(text_input)

            # Normalize features
            image_features /= image_features.norm(dim=-1, keepdim=True)
            text_features /= text_features.norm(dim=-1, keepdim=True)

            # Compute cosine similarity
            similarity = (image_features @ text_features.T).item()

        return similarity
```

## Exercise 2: Object Identification and Grounding

### Task 2.1: Object Detection System

Implement object detection for vision-language grounding:

```python
from ultralytics import YOLO

class ObjectDetectionSystem:
    def __init__(self, model_path="yolov8n.pt"):
        """
        Initialize object detection system
        """
        self.model = YOLO(model_path)
        self.confidence_threshold = 0.5
        self.classes = self.model.names

    def detect_objects(self, image):
        """
        Detect objects in an image
        """
        results = self.model(image, conf=self.confidence_threshold)

        detections = []
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = box.conf[0].cpu().numpy()
                    cls = int(box.cls[0].cpu().numpy())

                    detections.append({
                        "bbox": [int(x1), int(y1), int(x2), int(y2)],
                        "confidence": float(conf),
                        "class_id": cls,
                        "class_name": self.classes[cls]
                    })

        return detections
```

### Task 2.2: Language-Guided Object Grounding

Implement language-guided object grounding:

```python
class LanguageGuidedGrounding:
    def __init__(self):
        self.clip_model, self.preprocess = clip.load("ViT-B/32")
        self.clip_model.eval()

    def ground_text_to_objects(self, image, text_descriptions, detections):
        """
        Ground text descriptions to detected objects
        """
        # Encode the text descriptions
        text_tokens = clip.tokenize(text_descriptions)

        with torch.no_grad():
            text_features = self.clip_model.encode_text(text_tokens)
            text_features /= text_features.norm(dim=-1, keepdim=True)

        results = []
        for detection in detections:
            # Crop object from image
            x1, y1, x2, y2 = detection["bbox"]
            cropped_obj = image[y1:y2, x1:x2]

            if cropped_obj.size == 0:
                continue

            # Preprocess and encode the cropped object
            pil_image = Image.fromarray(cropped_obj)
            image_input = self.preprocess(pil_image).unsqueeze(0)

            with torch.no_grad():
                image_features = self.clip_model.encode_image(image_input)
                image_features /= image_features.norm(dim=-1, keepdim=True)

            # Compute similarities with all text descriptions
            similarities = (image_features @ text_features.T).squeeze().cpu().numpy()

            # Find the best matching description
            best_match_idx = np.argmax(similarities)
            best_similarity = similarities[best_match_idx]

            results.append({
                "detection": detection,
                "best_match": text_descriptions[best_match_idx],
                "similarity": float(best_similarity)
            })

        return results
```

## Exercise 3: Scene Understanding

### Task 3.1: Scene Graph Generation

Implement scene graph generation:

```python
import numpy as np

class SceneGraphGenerator:
    def generate_scene_graph(self, detections):
        """
        Generate a scene graph from object detections
        """
        # Create nodes for each object
        nodes = []
        for i, detection in enumerate(detections):
            nodes.append({
                "id": i,
                "class_name": detection["class_name"],
                "bbox": detection["bbox"],
                "center": [(detection["bbox"][0] + detection["bbox"][2]) // 2,
                          (detection["bbox"][1] + detection["bbox"][3]) // 2]
            })

        # Create edges based on spatial relationships
        edges = []
        for i, node1 in enumerate(nodes):
            for j, node2 in enumerate(nodes):
                if i != j:
                    center1 = node1["center"]
                    center2 = node2["center"]

                    # Calculate distance
                    dx = center2[0] - center1[0]
                    dy = center2[1] - center1[1]
                    distance = np.sqrt(dx**2 + dy**2)

                    # Determine spatial relationship
                    if abs(dx) > abs(dy):
                        relationship_type = "right" if dx > 0 else "left"
                    else:
                        relationship_type = "below" if dy > 0 else "above"

                    edges.append({
                        "subject": i,
                        "object": j,
                        "relationship": relationship_type,
                        "distance": float(distance)
                    })

        return {"nodes": nodes, "edges": edges}
```

## Exercise 4: ROS 2 Integration

### Task 4.1: Vision-Language ROS 2 Node

Create a ROS 2 node that integrates the vision-language system:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json

class VisionLanguageNode(Node):
    def __init__(self):
        super().__init__('vision_language_node')

        # Initialize components
        self.vision_system = ObjectDetectionSystem()
        self.grounding_system = LanguageGuidedGrounding()
        self.scene_graph_generator = SceneGraphGenerator()
        self.bridge = CvBridge()

        # Create subscribers
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )

        self.language_subscriber = self.create_subscription(
            String,
            '/natural_language_input',
            self.language_callback,
            10
        )

        # Create publishers
        self.grounded_objects_publisher = self.create_publisher(
            String,
            '/vision_language/grounded_objects',
            10
        )

        self.scene_graph_publisher = self.create_publisher(
            String,
            '/vision_language/scene_graph',
            10
        )

        self.get_logger().info('Vision-Language Node initialized')

    def image_callback(self, msg):
        """
        Handle incoming image messages
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Detect objects in the image
            detections = self.vision_system.detect_objects(cv_image)

            if detections:
                # Generate scene graph
                scene_graph = self.scene_graph_generator.generate_scene_graph(detections)

                # Publish scene graph
                scene_graph_msg = String()
                scene_graph_msg.data = json.dumps(scene_graph)
                self.scene_graph_publisher.publish(scene_graph_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def language_callback(self, msg):
        """
        Handle incoming language messages
        """
        try:
            query = msg.data
            self.get_logger().info(f'Received language query: {query}')

            # This would be connected to image processing in a real implementation
            # For this exercise, we just acknowledge the query

        except Exception as e:
            self.get_logger().error(f'Error processing language: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = VisionLanguageNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Validation of Vision-Language System

### Grounding Validation

Validate that the vision-language system correctly identifies and grounds objects:

```python
def validate_grounding_performance(grounding_results, expected_groundings):
    """
    Validate grounding performance against expected results
    """
    validation_result = {
        "accuracy": 0.0,
        "correct_groundings": 0,
        "total_groundings": len(grounding_results),
        "expected_count": len(expected_groundings)
    }

    if not grounding_results or not expected_groundings:
        return validation_result

    # Count correct groundings
    correct = 0
    for result in grounding_results:
        for expected in expected_groundings:
            # Simple string matching for demonstration
            if (result["best_match"].lower() in expected["description"].lower() or
                expected["description"].lower() in result["best_match"].lower()):
                correct += 1
                break

    validation_result["correct_groundings"] = correct
    validation_result["accuracy"] = correct / len(expected_groundings) if expected_groundings else 0

    return validation_result
```

## Assessment Rubric

### Technical Implementation (60 points)
- [ ] Vision-language system setup (15 points)
- [ ] Object detection and identification (15 points)
- [ ] Language-guided grounding (15 points)
- [ ] ROS 2 integration (15 points)

### Functionality (25 points)
- [ ] Object identification accuracy (10 points)
- [ ] Grounding effectiveness (10 points)
- [ ] System responsiveness (5 points)

### Code Quality (15 points)
- [ ] Code organization and structure (5 points)
- [ ] Error handling and validation (5 points)
- [ ] Documentation and comments (5 points)

## Summary

This exercise provides hands-on experience with vision-language integration in VLA systems. Students implement a complete pipeline from visual object detection to language grounding, with integration into ROS 2 systems.