# Implementing Whisper-Based Voice Pipelines

## Introduction

OpenAI Whisper is a state-of-the-art automatic speech recognition (ASR) system ideal for voice-controlled robotic applications. This chapter provides instructions for implementing Whisper-based voice pipelines in Vision-Language-Action (VLA) systems, enabling robots to understand and respond to natural spoken commands.

## Prerequisites

Before implementing Whisper-based voice pipelines, ensure the following:

### Hardware Requirements
- **GPU**: NVIDIA GPU with CUDA support (recommended: RTX 3060 or higher)
- **Memory**: Minimum 8GB RAM, 16GB recommended for real-time processing
- **Audio Input**: USB microphone with 16kHz sampling rate support

### Software Requirements
- **Python**: Version 3.8 or higher
- **CUDA**: Version 11.2 or higher (for GPU acceleration)
- **ROS 2**: Humble Hawksbill or later for robotic integration

### Python Dependencies
```bash
pip install openai-whisper
pip install torch torchvision torchaudio
pip install pyaudio
```

## Installing and Setting Up Whisper

### Basic Installation

Install Whisper using pip:

```bash
pip install -U openai-whisper
```

For GPU acceleration, ensure you have the appropriate PyTorch version:

```bash
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

### Model Selection

Whisper offers several model sizes:

```python
import whisper

# Available models (ordered by size and quality)
models = {
    "tiny": "Fastest, smallest model (~39M parameters)",
    "base": "Good balance of speed and accuracy (~74M parameters)",
    "small": "Better quality, moderate speed (~244M parameters)",
    "medium": "High quality, slower (~769M parameters)",
    "large": "Best quality, slowest (~1550M parameters)"
}

# Load the base model for robotic applications
model = whisper.load_model("base")
```

For robotic applications, the "base" or "small" models provide the best balance between accuracy and real-time performance.

## Basic Whisper Implementation

### Simple Speech Recognition

Basic Whisper implementation:

```python
import whisper
import torch
import pyaudio
import wave
import numpy as np
import time

class WhisperVoicePipeline:
    def __init__(self, model_size="base", device="cuda" if torch.cuda.is_available() else "cpu"):
        """
        Initialize Whisper voice pipeline
        """
        self.device = device
        self.model = whisper.load_model(model_size).to(device)
        print(f"Loaded Whisper {model_size} model on {device}")

        # Audio configuration
        self.rate = 16000  # Sampling rate
        self.chunk = 1024  # Audio chunk size
        self.format = pyaudio.paInt16
        self.channels = 1

    def record_audio(self, duration=5):
        """
        Record audio from microphone for specified duration
        """
        p = pyaudio.PyAudio()

        stream = p.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        print(f"Recording for {duration} seconds...")
        frames = []

        for i in range(0, int(self.rate / self.chunk * duration)):
            data = stream.read(self.chunk)
            frames.append(data)

        print("Recording finished.")

        stream.stop_stream()
        stream.close()
        p.terminate()

        # Save to WAV file for processing
        filename = f"temp_recording_{int(time.time())}.wav"
        wf = wave.open(filename, 'wb')
        wf.setnchannels(self.channels)
        wf.setsampwidth(p.get_sample_size(self.format))
        wf.setframerate(self.rate)
        wf.writeframes(b''.join(frames))
        wf.close()

        return filename

    def transcribe_audio(self, audio_file):
        """
        Transcribe audio file using Whisper
        """
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

# Example usage
if __name__ == "__main__":
    pipeline = WhisperVoicePipeline(model_size="base")
    audio_file = pipeline.record_audio(duration=5)
    result = pipeline.transcribe_audio(audio_file)

    print(f"Transcribed text: {result['text']}")
    print(f"Detected language: {result['language']}")
    print(f"Confidence: {result['confidence']}")
```

## Real-Time Voice Pipeline Implementation

### Streaming Voice Recognition

For real-time robotic applications, implement a streaming voice recognition system:

```python
import threading
import queue
import time
from dataclasses import dataclass
from typing import Optional

@dataclass
class VoiceCommand:
    text: str
    confidence: float
    timestamp: float
    language: str

class StreamingWhisperPipeline:
    def __init__(self, model_size="base", device="cuda" if torch.cuda.is_available() else "cpu"):
        """
        Initialize streaming Whisper voice pipeline
        """
        self.device = device
        self.model = whisper.load_model(model_size).to(device)

        # Audio configuration
        self.rate = 16000
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1

        # Voice activity detection parameters
        self.silence_threshold = 500
        self.silence_duration = 1.0  # seconds of silence to trigger recognition
        self.min_speech_duration = 0.5  # minimum speech duration

        # Processing queues
        self.audio_queue = queue.Queue()
        self.command_queue = queue.Queue()

        # State management
        self.is_recording = False
        self.recording_thread = None
        self.processing_thread = None

    def start_listening(self):
        """
        Start listening for voice commands
        """
        self.is_recording = True
        self.recording_thread = threading.Thread(target=self._record_audio)
        self.processing_thread = threading.Thread(target=self._process_audio)

        self.recording_thread.start()
        self.processing_thread.start()

        print("Started listening for voice commands...")

    def stop_listening(self):
        """
        Stop listening for voice commands
        """
        self.is_recording = False
        if self.recording_thread:
            self.recording_thread.join()
        if self.processing_thread:
            self.processing_thread.join()

        print("Stopped listening for voice commands.")

    def _record_audio(self):
        """
        Record audio in a separate thread
        """
        p = pyaudio.PyAudio()

        stream = p.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        # Buffer to hold audio chunks
        audio_buffer = []
        silence_start = None
        speech_detected = False

        while self.is_recording:
            data = stream.read(self.chunk)
            audio_buffer.append(data)

            # Convert to numpy array for analysis
            audio_array = np.frombuffer(data, dtype=np.int16)
            amplitude = np.abs(audio_array).mean()

            if amplitude > self.silence_threshold:
                # Speech detected
                speech_detected = True
                silence_start = None
            else:
                # Silence detected
                if speech_detected and silence_start is None:
                    silence_start = time.time()
                elif speech_detected and (time.time() - silence_start) >= self.silence_duration:
                    # End of speech detected
                    if len(audio_buffer) > (self.min_speech_duration * self.rate / self.chunk):
                        # Send audio for processing
                        audio_data = b''.join(audio_buffer)
                        self.audio_queue.put(audio_data)

                    # Reset for next utterance
                    audio_buffer = []
                    speech_detected = False
                    silence_start = None

        stream.stop_stream()
        stream.close()
        p.terminate()

    def _process_audio(self):
        """
        Process audio chunks and perform transcription
        """
        while self.is_recording or not self.audio_queue.empty():
            try:
                # Wait for audio data with timeout
                audio_data = self.audio_queue.get(timeout=0.1)

                # Create temporary WAV file
                temp_filename = f"temp_{int(time.time() * 1000)}.wav"
                wf = wave.open(temp_filename, 'wb')
                wf.setnchannels(self.channels)
                wf.setsampwidth(pyaudio.PyAudio().get_sample_size(self.format))
                wf.setframerate(self.rate)
                wf.writeframes(audio_data)
                wf.close()

                # Transcribe the audio
                result = self.transcribe_audio(temp_filename)

                if result and result['confidence'] > -0.5:  # Filter low-confidence results
                    command = VoiceCommand(
                        text=result['text'],
                        confidence=result['confidence'],
                        timestamp=time.time(),
                        language=result['language']
                    )
                    self.command_queue.put(command)
                    print(f"Recognized: {command.text} (Confidence: {command.confidence:.2f})")

                # Clean up temporary file
                import os
                os.remove(temp_filename)

            except queue.Empty:
                continue
            except Exception as e:
                print(f"Error processing audio: {e}")

    def transcribe_audio(self, audio_file):
        """
        Transcribe audio file using Whisper with real-time optimizations
        """
        try:
            # Load audio and pad/trim it
            audio = whisper.load_audio(audio_file)
            audio = whisper.pad_or_trim(audio)

            # Convert to log-Mel spectrogram
            mel = whisper.log_mel_spectrogram(audio).to(self.model.device)

            # Detect language
            _, probs = self.model.detect_language(mel)
            detected_language = max(probs, key=probs.get)

            # Decode the audio with faster settings
            options = whisper.DecodingOptions(
                fp16=torch.cuda.is_available(),
                language=detected_language,
                without_timestamps=True,
                max_initial_timestamp=1.0
            )

            result = whisper.decode(self.model, mel, options)

            return {
                "text": result.text.strip(),
                "language": detected_language,
                "confidence": result.avg_logprob
            }
        except Exception as e:
            print(f"Transcription error: {e}")
            return None

    def get_command(self) -> Optional[VoiceCommand]:
        """
        Get the next voice command from the queue
        """
        try:
            return self.command_queue.get_nowait()
        except queue.Empty:
            return None

# Example usage
if __name__ == "__main__":
    pipeline = StreamingWhisperPipeline(model_size="base")
    pipeline.start_listening()

    try:
        # Listen for 30 seconds
        start_time = time.time()
        while time.time() - start_time < 30:
            command = pipeline.get_command()
            if command:
                print(f"Command received: {command.text}")
                # Process the command here
                # For example, send to ROS 2 topic or process as robot command
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop_listening()
```

## Integration with ROS 2

### ROS 2 Node Implementation

Create a ROS 2 node that integrates the Whisper pipeline:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import AudioData
import threading
import queue

class WhisperROSNode(Node):
    def __init__(self):
        super().__init__('whisper_voice_node')

        # Initialize Whisper pipeline
        self.pipeline = StreamingWhisperPipeline(model_size="base")

        # Create publishers
        self.text_publisher = self.create_publisher(String, 'speech_text', 10)
        self.confidence_publisher = self.create_publisher(Float32, 'speech_confidence', 10)
        self.command_publisher = self.create_publisher(String, 'voice_command', 10)

        # Create service to start/stop recognition
        self.start_service = self.create_service(
            Empty, 'start_voice_recognition', self.start_recognition_callback
        )
        self.stop_service = self.create_service(
            Empty, 'stop_voice_recognition', self.stop_recognition_callback
        )

        # Start voice recognition
        self.pipeline.start_listening()

        # Timer to process commands
        self.timer = self.create_timer(0.1, self.process_commands)

        self.get_logger().info('Whisper voice recognition node started')

    def start_recognition_callback(self, request, response):
        self.pipeline.start_listening()
        self.get_logger().info('Voice recognition started')
        return response

    def stop_recognition_callback(self, request, response):
        self.pipeline.stop_listening()
        self.get_logger().info('Voice recognition stopped')
        return response

    def process_commands(self):
        command = self.pipeline.get_command()
        if command:
            # Publish recognized text
            text_msg = String()
            text_msg.data = command.text
            self.text_publisher.publish(text_msg)

            # Publish confidence
            confidence_msg = Float32()
            confidence_msg.data = float(command.confidence)
            self.confidence_publisher.publish(confidence_msg)

            # Process command and publish as voice command
            # This could involve NLP processing to extract robot commands
            processed_command = self.process_voice_command(command.text)
            if processed_command:
                cmd_msg = String()
                cmd_msg.data = processed_command
                self.command_publisher.publish(cmd_msg)

    def process_voice_command(self, text):
        """
        Process voice command text and extract robot commands
        """
        # Simple command processing - in practice, this would involve
        # more sophisticated NLP and intent classification
        text_lower = text.lower().strip()

        # Example command patterns
        if 'move forward' in text_lower:
            return 'move_forward'
        elif 'turn left' in text_lower:
            return 'turn_left'
        elif 'turn right' in text_lower:
            return 'turn_right'
        elif 'stop' in text_lower:
            return 'stop'
        elif 'help' in text_lower:
            return 'help'
        else:
            # Return the original text if no specific command is recognized
            return text_lower

def main(args=None):
    rclpy.init(args=args)
    node = WhisperROSNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline.stop_listening()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Troubleshooting and Best Practices

### Common Issues and Solutions

1. **High Latency**: Use smaller models ("base" or "small") for real-time applications
2. **Low Accuracy**: Fine-tune on domain-specific data or use language-specific models
3. **Memory Issues**: Implement proper memory management and model reloading

### Best Practices

1. **Model Selection**: Choose the smallest model that meets accuracy requirements
2. **Error Handling**: Implement robust error handling and fallback mechanisms
3. **Privacy**: Process sensitive audio locally when possible

## Summary

Implementing Whisper-based voice pipelines in VLA systems requires careful consideration of real-time performance, accuracy, and integration with robotic systems. By following the patterns in this guide, developers can create robust voice-to-action systems that enable natural human-robot interaction.