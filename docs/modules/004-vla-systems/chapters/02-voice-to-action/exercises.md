# Voice Command Pipeline Implementation Exercises

## Exercise Overview

This exercise provides hands-on experience with implementing voice-to-action pipelines in Vision-Language-Action (VLA) systems. Students will build and test a complete voice command pipeline using OpenAI Whisper and integrate it with robotic systems through ROS 2.

## Exercise Objectives

By completing this exercise, students will:

1. Implement a Whisper-based speech recognition pipeline
2. Integrate the pipeline with ROS 2 communication systems
3. Develop intent classification and action mapping systems
4. Test and validate the voice command pipeline performance

## Exercise 1: Basic Whisper Implementation

### Task 1.1: Set up Whisper Environment

Create a Python environment with Whisper and test basic functionality:

```python
import whisper
import torch

def test_whisper_installation():
    """
    Test basic Whisper functionality
    """
    print("Testing Whisper installation...")

    # Check if CUDA is available
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Using device: {device}")

    # Load a small model for testing
    try:
        model = whisper.load_model("tiny", device=device)
        print("Whisper model loaded successfully!")
        print("Installation test completed successfully!")
    except Exception as e:
        print(f"Error loading Whisper model: {e}")
        return False

    return True

if __name__ == "__main__":
    success = test_whisper_installation()
    if success:
        print("✓ Whisper environment setup completed")
    else:
        print("✗ Whisper environment setup failed")
```

### Task 1.2: Audio Recording and Processing

Implement audio recording and preprocessing functionality:

```python
import pyaudio
import wave
import numpy as np
import whisper
import torch
import time

class AudioRecorder:
    def __init__(self):
        self.rate = 16000  # Sampling rate
        self.chunk = 1024  # Audio chunk size
        self.format = pyaudio.paInt16
        self.channels = 1
        self.silence_threshold = 500

    def record_until_silence(self, max_duration=10, filename=None):
        """
        Record audio until silence is detected
        """
        if filename is None:
            filename = f"voice_command_{int(time.time())}.wav"

        p = pyaudio.PyAudio()

        stream = p.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        print("Recording... Speak now (recording will stop after silence)")
        frames = []
        silence_frames = 0
        max_silence_frames = int(1.0 * self.rate / self.chunk)  # 1 second of silence

        for i in range(0, int(self.rate / self.chunk * max_duration)):
            data = stream.read(self.chunk)
            frames.append(data)

            # Check for silence
            audio_array = np.frombuffer(data, dtype=np.int16)
            amplitude = np.abs(audio_array).mean()

            if amplitude < self.silence_threshold:
                silence_frames += 1
                if silence_frames >= max_silence_frames:
                    print("Silence detected, stopping recording...")
                    break
            else:
                silence_frames = 0  # Reset silence counter

        print("Recording finished.")

        stream.stop_stream()
        stream.close()
        p.terminate()

        # Save to WAV file
        wf = wave.open(filename, 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(p.get_sample_size(self.format))
        wf.setframerate(self.rate)
        wf.writeframes(b''.join(frames))
        wf.close()

        return filename
```

### Task 1.3: Whisper Transcription Integration

Integrate Whisper with the audio recorder:

```python
import whisper
import torch

class WhisperTranscriber:
    def __init__(self, model_size="base"):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = whisper.load_model(model_size).to(self.device)
        self.recorder = AudioRecorder()  # Assume AudioRecorder is defined
        print(f"Loaded Whisper {model_size} model on {self.device}")

    def transcribe_audio(self, audio_file):
        """
        Transcribe audio file using Whisper
        """
        try:
            # Load audio and pad/trim it to fit 30-second context
            audio = whisper.load_audio(audio_file)
            audio = whisper.pad_or_trim(audio)

            # Convert to log-Mel spectrogram
            mel = whisper.log_mel_spectrogram(audio).to(self.model.device)

            # Detect language
            _, probs = self.model.detect_language(mel)
            detected_language = max(probs, key=probs.get)

            # Decode the audio
            options = whisper.DecodingOptions(fp16=torch.cuda.is_available())
            result = whisper.decode(self.model, mel, options)

            return {
                "text": result.text,
                "language": detected_language,
                "confidence": result.avg_logprob
            }
        except Exception as e:
            print(f"Error during transcription: {e}")
            return None

    def transcribe_live(self):
        """
        Record and transcribe live voice input
        """
        print("Recording voice command...")
        audio_file = self.recorder.record_until_silence(max_duration=10)

        print("Transcribing...")
        result = self.transcribe_audio(audio_file)

        if result:
            print(f"Transcribed text: {result['text']}")
            print(f"Confidence: {result['confidence']:.2f}")
            return result
        else:
            print("Transcription failed")
            return None
```

## Exercise 2: Intent Classification System

### Task 2.1: Basic Intent Classifier

Create a simple intent classification system:

```python
import re

class IntentClassifier:
    def __init__(self):
        # Define intent patterns
        self.intents = {
            "move_forward": {
                "patterns": [r"go forward", r"move forward", r"go straight", r"move straight", r"forward"],
                "confidence_threshold": 0.8
            },
            "turn_left": {
                "patterns": [r"turn left", r"go left", r"left", r"turn to the left"],
                "confidence_threshold": 0.8
            },
            "navigate_to_location": {
                "patterns": [r"go to (.+)", r"navigate to (.+)"],
                "confidence_threshold": 0.7
            },
            "pick_up_object": {
                "patterns": [r"pick up (.+)", r"grasp (.+)", r"grab (.+)"],
                "confidence_threshold": 0.7
            },
            "stop": {
                "patterns": [r"stop", r"halt", r"freeze", r"pause"],
                "confidence_threshold": 0.9
            }
        }

    def classify_intent(self, text: str) -> dict:
        """
        Classify intent from text input
        """
        text_lower = text.lower().strip()
        best_match = None
        best_confidence = 0.0
        extracted_params = {}

        for intent_name, intent_data in self.intents.items():
            for pattern in intent_data["patterns"]:
                match = re.search(pattern, text_lower)
                if match:
                    # Calculate confidence based on pattern match
                    confidence = min(1.0, len(pattern) / len(text_lower) if len(text_lower) > 0 else 1.0)

                    # Extract parameters if pattern has groups
                    if match.groups():
                        extracted_params = {"target": match.group(1)}

                    if confidence > best_confidence:
                        best_confidence = confidence
                        best_match = {
                            "intent": intent_name,
                            "confidence": confidence,
                            "parameters": extracted_params.copy()
                        }

        # Return result or default to unknown intent
        if best_match and best_match["confidence"] >= self.intents[best_match["intent"]]["confidence_threshold"]:
            return best_match
        else:
            return {
                "intent": "unknown",
                "confidence": 0.0,
                "parameters": {}
            }

# Test the intent classifier
if __name__ == "__main__":
    classifier = IntentClassifier()

    test_commands = [
        "Go forward",
        "Turn left",
        "Go to the kitchen",
        "Pick up the red cup",
        "Stop immediately"
    ]

    print("Testing intent classification:")
    for command in test_commands:
        result = classifier.classify_intent(command)
        print(f"Command: '{command}' -> Intent: {result['intent']} (Confidence: {result['confidence']:.2f})")
```

## Exercise 3: ROS 2 Integration

### Task 3.1: ROS 2 Voice Node Setup

Create a ROS 2 node that integrates voice recognition:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        # Initialize Whisper transcriber and intent classifier
        self.transcriber = WhisperTranscriber(model_size="base")  # Assume defined
        self.intent_classifier = IntentClassifier()  # Assume defined

        # Create publishers
        self.text_publisher = self.create_publisher(String, 'speech_text', 10)
        self.intent_publisher = self.create_publisher(String, 'voice_intent', 10)
        self.command_publisher = self.create_publisher(String, 'robot_command', 10)

        # Create timer to periodically check for voice commands
        self.timer = self.create_timer(1.0, self.check_for_voice)

        # State management
        self.waiting_for_audio = False

        self.get_logger().info('Voice Command Node initialized')

    def check_for_voice(self):
        """
        Check for voice commands (in a real implementation, this would be triggered by audio detection)
        """
        if not self.waiting_for_audio:
            # For this exercise, we'll simulate voice detection
            self.get_logger().info('Say a command now (timeout in 5 seconds)')
            self.waiting_for_audio = True

            # In a real system, this would be triggered by voice activity detection
            self.process_voice_command()

    def process_voice_command(self):
        """
        Process a voice command from the user
        """
        try:
            # Transcribe the voice command
            result = self.transcriber.transcribe_live()

            if result and result['confidence'] > -0.5:  # Reasonable confidence threshold
                text = result['text']

                # Publish the recognized text
                text_msg = String()
                text_msg.data = text
                self.text_publisher.publish(text_msg)

                # Classify the intent
                intent_result = self.intent_classifier.classify_intent(text)

                # Publish the intent
                intent_msg = String()
                intent_msg.data = f"{intent_result['intent']}:{intent_result['confidence']}"
                self.intent_publisher.publish(intent_msg)

                # Log the results
                self.get_logger().info(f'Recognized: {text}')
                self.get_logger().info(f'Intent: {intent_result["intent"]} (Confidence: {intent_result["confidence"]:.2f})')

                # Convert intent to robot command and publish
                robot_command = self.convert_intent_to_command(intent_result)
                if robot_command:
                    cmd_msg = String()
                    cmd_msg.data = robot_command
                    self.command_publisher.publish(cmd_msg)
                    self.get_logger().info(f'Sent robot command: {robot_command}')

                self.waiting_for_audio = False

            else:
                self.get_logger().info('No speech detected or low confidence')
                self.waiting_for_audio = False

        except Exception as e:
            self.get_logger().error(f'Error processing voice command: {e}')
            self.waiting_for_audio = False

    def convert_intent_to_command(self, intent_result: dict) -> str:
        """
        Convert intent to robot command string
        """
        intent = intent_result['intent']
        params = intent_result['parameters']

        if intent == 'move_forward':
            return 'MOVE_FORWARD'
        elif intent == 'turn_left':
            return 'TURN_LEFT'
        elif intent == 'navigate_to_location':
            location = params.get('target', 'unknown')
            return f'NAVIGATE_TO:{location}'
        elif intent == 'pick_up_object':
            obj = params.get('target', 'unknown')
            return f'PICK_UP:{obj}'
        elif intent == 'stop':
            return 'STOP'
        else:
            return f'UNKNOWN_COMMAND:{intent}'

def main(args=None):
    rclpy.init(args=args)

    node = VoiceCommandNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Assessment Rubric

### Technical Implementation (60 points)
- [ ] Whisper model integration (15 points)
- [ ] Audio recording and processing (15 points)
- [ ] Intent classification system (15 points)
- [ ] ROS 2 integration (15 points)

### Functionality (25 points)
- [ ] Voice command recognition accuracy (10 points)
- [ ] Intent classification precision (10 points)
- [ ] System responsiveness (5 points)

### Code Quality (15 points)
- [ ] Code organization and structure (5 points)
- [ ] Error handling and validation (5 points)
- [ ] Documentation and comments (5 points)

## Summary

This exercise provides hands-on experience with voice-to-action pipeline implementation in VLA systems. Students gain practical experience with speech recognition, natural language processing, intent classification, and robotic integration.