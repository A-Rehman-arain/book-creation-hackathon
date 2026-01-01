---
sidebar_position: 6
title: Voice-to-Action using speech recognition (OpenAI Whisper)
---

# Voice-to-Action using speech recognition (OpenAI Whisper)

Voice-to-Action systems convert human speech into executable robot commands, enabling natural interaction between humans and robots. Using OpenAI Whisper for speech recognition, these systems provide accurate transcription of spoken language into text that can be processed by robotic control systems.

## Learning Objectives

After completing this chapter, you will understand:
- The core concepts of voice-to-action conversion with OpenAI Whisper
- The speech recognition pipeline and processing steps
- How to implement voice command processing for robotics
- The voice-to-action translation process

## Introduction to Voice-to-Action Systems

Voice-to-Action systems bridge the gap between natural human communication and robot execution by converting spoken commands into structured robot actions. These systems typically follow this pipeline:

- **Audio Input**: Capturing human speech from the environment
- **Speech Recognition**: Converting audio to text using models like OpenAI Whisper
- **Natural Language Processing**: Interpreting the meaning of the text command
- **Command Mapping**: Converting the interpreted command into robot actions
- **Execution**: Executing the mapped actions on the robot

### Key Components

A voice-to-action system consists of several key components:

1. **Audio Capture**: Microphones and audio processing for speech input
2. **Speech Recognition Engine**: Transcribes audio to text (e.g., OpenAI Whisper)
3. **Language Understanding**: Processes text to extract intent and parameters
4. **Command Mapper**: Maps understood commands to robot actions
5. **Execution Interface**: Sends commands to the robot control system

## OpenAI Whisper for Speech Recognition

OpenAI Whisper is a state-of-the-art speech recognition model that provides accurate transcription of speech to text. It supports multiple languages and can be used for various speech recognition tasks.

### Whisper Model Variants

Whisper comes in several model sizes, each with different performance and accuracy characteristics:

- **tiny**: Fastest but least accurate, suitable for real-time applications
- **base**: Good balance of speed and accuracy
- **small**: Better accuracy with moderate computational requirements
- **medium**: High accuracy suitable for most applications
- **large**: Highest accuracy but computationally intensive

```python
import whisper
import torch

# Load the Whisper model
model = whisper.load_model("base")

# Transcribe an audio file
result = model.transcribe("robot_command.wav")

# Get the transcribed text
command_text = result["text"]
print(f"Recognized command: {command_text}")
```

### Real-time Processing

For real-time voice-to-action systems, you can use Whisper in streaming mode:

```python
import whisper
import pyaudio
import wave
import numpy as np
from queue import Queue

class RealTimeVoiceToAction:
    def __init__(self, model_size="base"):
        self.model = whisper.load_model(model_size)
        self.audio_queue = Queue()

        # Audio parameters
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000
        self.chunk = 1024

    def start_listening(self):
        """Start listening for audio input"""
        p = pyaudio.PyAudio()

        stream = p.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        print("Listening for voice commands...")

        while True:
            data = stream.read(self.chunk)
            self.audio_queue.put(data)

            # Process audio in chunks when we have enough data
            if self.audio_queue.qsize() > 10:  # Process every 10 chunks
                self.process_audio_chunk()

    def process_audio_chunk(self):
        """Process a chunk of audio data"""
        # Collect audio frames
        frames = []
        while not self.audio_queue.empty():
            frames.append(self.audio_queue.get())

        if len(frames) > 0:
            # Convert to numpy array
            audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)

            # Convert to float32 and normalize
            audio_array = audio_data.astype(np.float32) / 32768.0

            # Transcribe using Whisper
            result = self.model.transcribe(audio_array)

            if result["text"].strip():  # Only process non-empty results
                self.process_command(result["text"])

    def process_command(self, command_text):
        """Process the recognized command text"""
        print(f"Recognized: {command_text}")

        # Here you would implement your command interpretation logic
        action = self.interpret_command(command_text)
        if action:
            print(f"Executing action: {action}")
            # Execute the action on the robot
            self.execute_robot_action(action)

    def interpret_command(self, text):
        """Interpret the command text and map to robot actions"""
        text_lower = text.lower().strip()

        if "move forward" in text_lower:
            return {"action": "move", "direction": "forward", "distance": 1.0}
        elif "turn left" in text_lower:
            return {"action": "turn", "direction": "left", "angle": 90}
        elif "pick up" in text_lower or "grasp" in text_lower:
            return {"action": "grasp", "target": self.extract_object(text_lower)}
        elif "stop" in text_lower:
            return {"action": "stop"}
        else:
            return None

    def extract_object(self, text):
        """Extract object from command text"""
        # Simple object extraction (in practice, this would be more sophisticated)
        words = text.split()
        for i, word in enumerate(words):
            if word in ["cup", "box", "ball", "object"]:
                return word
        return "unknown_object"

    def execute_robot_action(self, action):
        """Execute the action on the robot"""
        # In a real implementation, this would interface with ROS 2
        print(f"Robot executing: {action}")

# Example usage
if __name__ == "__main__":
    vta_system = RealTimeVoiceToAction(model_size="base")
    vta_system.start_listening()
```

## Speech Recognition Pipeline

### Audio Preprocessing

Before feeding audio to the Whisper model, preprocessing is often required:

- **Noise Reduction**: Remove background noise to improve recognition
- **Normalization**: Normalize audio levels for consistent processing
- **Format Conversion**: Convert to the required format for the model

### Command Interpretation

After speech recognition, the text needs to be interpreted to extract meaningful commands:

- **Intent Recognition**: Identify the type of action requested
- **Parameter Extraction**: Extract parameters like targets, directions, etc.
- **Validation**: Validate that the command is safe and executable

## Integration with Robotic Systems

### ROS 2 Integration

Voice-to-action systems can be integrated with ROS 2 for robotic control:

```yaml
# Example ROS 2 service definition for voice commands
# VoiceCommand.srv
string command_text
---
bool success
string[] execution_log
```

### Command Mapping Strategies

Different strategies can be used to map recognized commands to robot actions:

#### Template-Based Mapping
- Use predefined templates for common commands
- Extract parameters from the recognized text
- Map to specific robot actions

#### Semantic Parsing
- Parse the command text for semantic meaning
- Use natural language understanding to extract intent
- Generate appropriate action sequences

#### LLM-Based Interpretation
- Use large language models to interpret complex commands
- Generate action sequences based on context
- Handle ambiguous or complex requests

## Practical Example: Voice-to-Action System

Here's a complete example of a voice-to-action system integrated with a robot:

```python
import whisper
import pyaudio
import numpy as np
from dataclasses import dataclass
from typing import Optional, Dict, Any
import time

@dataclass
class RobotAction:
    action_type: str
    parameters: Dict[str, Any]
    confidence: float

class VoiceToActionSystem:
    def __init__(self, model_size="base"):
        self.model = whisper.load_model(model_size)
        self.robot_interface = RobotInterface()
        self.command_history = []

        # Audio configuration
        self.audio_format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000
        self.chunk = 1024
        self.audio_buffer = []

    def process_voice_command(self, audio_data: np.ndarray) -> Optional[RobotAction]:
        """
        Process audio data and convert to robot action
        """
        # Transcribe audio using Whisper
        result = self.model.transcribe(audio_data)
        recognized_text = result["text"].strip()

        if not recognized_text:
            return None

        print(f"Recognized: '{recognized_text}'")

        # Interpret the command
        action = self.interpret_command(recognized_text)

        if action:
            self.command_history.append({
                "timestamp": time.time(),
                "recognized_text": recognized_text,
                "action": action
            })

        return action

    def interpret_command(self, text: str) -> Optional[RobotAction]:
        """
        Interpret the recognized text and generate robot action
        """
        text_lower = text.lower()

        # Define command patterns
        command_patterns = [
            {
                "pattern": ["move", "forward"],
                "action": "move_base",
                "params": {"direction": "forward", "distance": 1.0}
            },
            {
                "pattern": ["move", "backward"],
                "action": "move_base",
                "params": {"direction": "backward", "distance": 1.0}
            },
            {
                "pattern": ["turn", "left"],
                "action": "rotate",
                "params": {"angle": 90, "direction": "counterclockwise"}
            },
            {
                "pattern": ["turn", "right"],
                "action": "rotate",
                "params": {"angle": 90, "direction": "clockwise"}
            },
            {
                "pattern": ["grasp", "pick"],
                "action": "grasp",
                "params": {"object": self.extract_object(text_lower)}
            },
            {
                "pattern": ["place", "put"],
                "action": "place",
                "params": {"location": self.extract_location(text_lower)}
            }
        ]

        # Match command to pattern
        for pattern in command_patterns:
            if all(word in text_lower for word in pattern["pattern"]):
                return RobotAction(
                    action_type=pattern["action"],
                    parameters=pattern["params"],
                    confidence=0.8  # Default confidence
                )

        # If no pattern matched, return None
        return None

    def extract_object(self, text: str) -> str:
        """
        Extract object from command text
        """
        objects = ["cup", "box", "bottle", "book", "ball", "object"]
        for obj in objects:
            if obj in text:
                return obj
        return "unknown_object"

    def extract_location(self, text: str) -> str:
        """
        Extract location from command text
        """
        locations = ["table", "shelf", "box", "container", "floor"]
        for loc in locations:
            if loc in text:
                return loc
        return "default_location"

    def execute_action(self, action: RobotAction) -> bool:
        """
        Execute the robot action
        """
        if action.confidence < 0.5:
            print(f"Action confidence too low: {action.confidence}")
            return False

        success = self.robot_interface.execute(action.action_type, action.parameters)

        if success:
            print(f"Successfully executed: {action.action_type} with {action.parameters}")
        else:
            print(f"Failed to execute: {action.action_type}")

        return success

class RobotInterface:
    """
    Simulated robot interface for demonstration
    """
    def execute(self, action_type: str, parameters: Dict[str, Any]) -> bool:
        """
        Execute action on the robot
        """
        print(f"Robot executing {action_type} with parameters: {parameters}")

        # Simulate action execution
        time.sleep(0.5)  # Simulate execution time

        # Return success (in practice, this would check actual robot response)
        return True

# Example usage
if __name__ == "__main__":
    # Create the voice-to-action system
    vta = VoiceToActionSystem(model_size="base")

    # Simulate audio processing (in practice, this would come from microphone)
    # For demonstration, we'll create some sample audio data
    print("Voice-to-Action system initialized")
    print("Ready to process voice commands...")

    # Example: Process a simulated command
    # In practice, you would continuously listen and process audio
    sample_command = "Please move forward and pick up the red cup"
    print(f"Processing sample command: '{sample_command}'")

    # In a real implementation, you would convert text to audio-like data
    # or capture actual audio from a microphone
    action = vta.interpret_command(sample_command)
    if action:
        vta.execute_action(action)
```

## Challenges and Considerations

### Audio Quality
- Background noise can affect recognition accuracy
- Microphone placement and quality are important
- Consider using noise cancellation techniques

### Real-time Performance
- Whisper models can be computationally intensive
- Balance accuracy with response time requirements
- Consider using smaller models for real-time applications

### Safety and Validation
- Validate commands before execution
- Implement safety checks and confirmation steps
- Handle ambiguous commands appropriately

## Future Directions

### Improved Recognition
- Fine-tuning Whisper for specific robotic commands
- Multi-modal recognition combining speech with other inputs
- Improved noise robustness for real-world environments

### Enhanced Understanding
- Better natural language understanding for complex commands
- Context-aware command interpretation
- Learning from user interactions and corrections

## Summary

Voice-to-Action systems using OpenAI Whisper provide a natural interface for human-robot interaction. By converting speech to text and then mapping that text to robot actions, these systems enable intuitive control of robotic systems. The key is balancing recognition accuracy with real-time performance while ensuring safety and reliability in robot execution.