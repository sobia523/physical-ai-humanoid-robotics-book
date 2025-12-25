#!/usr/bin/env python3
"""
Performance Benchmark Script for Digital Twin Optimization

This script measures various performance metrics for digital twin systems
including CPU usage, memory usage, simulation real-time factor, and
message processing rates. It can be used to evaluate the effectiveness
of optimization strategies.
"""

import time
import psutil
import subprocess
import argparse
import yaml
import json
import os
from datetime import datetime
from collections import deque
import matplotlib.pyplot as plt
import numpy as np

class PerformanceBenchmark:
    def __init__(self, config_file=None):
        self.config_file = config_file
        self.metrics = {
            'timestamp': [],
            'cpu_percent': [],
            'memory_percent': [],
            'memory_used_mb': [],
            'simulation_rtf': [],  # Real-time factor
            'message_rates': {},   # Topic-specific message rates
            'processing_times': {}, # Node-specific processing times
            'frame_rates': []      # Visualization frame rates
        }
        self.benchmark_data = []
        self.start_time = None
        self.duration = 0

        # Load configuration if provided
        if config_file and os.path.exists(config_file):
            with open(config_file, 'r') as f:
                self.config = yaml.safe_load(f)
        else:
            self.config = self._get_default_config()

    def _get_default_config(self):
        """Get default benchmark configuration"""
        return {
            'duration': 60,  # seconds
            'interval': 1.0,  # seconds between measurements
            'topics_to_monitor': [
                '/humanoid/lidar/scan',
                '/humanoid/imu/data',
                '/humanoid/camera/image_raw',
                '/humanoid/odom',
                '/humanoid/joint_states'
            ],
            'nodes_to_monitor': [
                'sensor_processor',
                'navigation',
                'localization',
                'robot_state_publisher'
            ],
            'performance_thresholds': {
                'cpu_warning': 80.0,
                'cpu_critical': 90.0,
                'memory_warning': 80.0,
                'memory_critical': 90.0,
                'rtf_warning': 0.8,
                'rtf_critical': 0.5
            }
        }

    def start_benchmark(self):
        """Start the performance benchmark"""
        print("Starting performance benchmark...")
        self.start_time = time.time()
        self.duration = self.config['duration']
        interval = self.config['interval']

        measurement_count = 0
        max_measurements = int(self.duration / interval)

        try:
            while measurement_count < max_measurements:
                self._collect_metrics()
                measurement_count += 1
                time.sleep(interval)

                # Print progress
                progress = (measurement_count / max_measurements) * 100
                print(f"Progress: {progress:.1f}% ({measurement_count}/{max_measurements})", end='\r')

        except KeyboardInterrupt:
            print("\nBenchmark interrupted by user")

        print(f"\nBenchmark completed after {time.time() - self.start_time:.1f} seconds")

    def _collect_metrics(self):
        """Collect performance metrics at current time"""
        timestamp = time.time()

        # CPU and memory metrics
        cpu_percent = psutil.cpu_percent(interval=None)
        memory_info = psutil.virtual_memory()
        memory_percent = memory_info.percent
        memory_used_mb = memory_info.used / (1024 * 1024)

        # Add to metrics
        self.metrics['timestamp'].append(timestamp)
        self.metrics['cpu_percent'].append(cpu_percent)
        self.metrics['memory_percent'].append(memory_percent)
        self.metrics['memory_used_mb'].append(memory_used_mb)

        # Collect simulation RTF (try to get from gz stats if available)
        rtf = self._get_simulation_rtf()
        self.metrics['simulation_rtf'].append(rtf)

        # Collect message rates for monitored topics
        topic_rates = self._get_topic_rates()
        for topic, rate in topic_rates.items():
            if topic not in self.metrics['message_rates']:
                self.metrics['message_rates'][topic] = []
            self.metrics['message_rates'][topic].append(rate)

        # Collect frame rates (if Unity is running and reporting)
        frame_rate = self._get_frame_rate()
        self.metrics['frame_rates'].append(frame_rate)

        # Store current measurement
        current_metrics = {
            'timestamp': timestamp,
            'cpu_percent': cpu_percent,
            'memory_percent': memory_percent,
            'memory_used_mb': memory_used_mb,
            'simulation_rtf': rtf,
            'topic_rates': topic_rates,
            'frame_rate': frame_rate
        }
        self.benchmark_data.append(current_metrics)

    def _get_simulation_rtf(self):
        """Get simulation real-time factor from Gazebo"""
        try:
            # Try to get RTF from gz stats
            result = subprocess.run(['gz', 'stats'], capture_output=True, text=True, timeout=2)
            if result.returncode == 0:
                # Parse RTF from output (format: "RTF[0.85]")
                output = result.stdout
                for line in output.split('\n'):
                    if 'RTF' in line:
                        # Extract RTF value
                        import re
                        match = re.search(r'RTF\[(\d+\.?\d*)\]', line)
                        if match:
                            return float(match.group(1))
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError, ValueError):
            pass

        # If gz stats is not available, return 0 (will be handled in analysis)
        return 0.0

    def _get_topic_rates(self):
        """Get message rates for monitored topics"""
        rates = {}
        for topic in self.config['topics_to_monitor']:
            try:
                # Use ros2 topic hz to get rate
                result = subprocess.run(['ros2', 'topic', 'hz', topic],
                                      capture_output=True, text=True, timeout=2)
                if result.returncode == 0:
                    # Parse the average rate from output
                    output = result.stdout
                    # Look for lines containing average rate
                    import re
                    match = re.search(r'average rate: (\d+\.?\d*)', output)
                    if match:
                        rates[topic] = float(match.group(1))
                    else:
                        rates[topic] = 0.0
                else:
                    rates[topic] = 0.0
            except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
                rates[topic] = 0.0

        return rates

    def _get_frame_rate(self):
        """Get frame rate from Unity or other visualization"""
        # This would typically read from a Unity-reported metric
        # For now, return a simulated value or 0 if not available
        return 0.0

    def analyze_results(self):
        """Analyze benchmark results and generate report"""
        print("\n" + "="*60)
        print("PERFORMANCE BENCHMARK RESULTS")
        print("="*60)

        if not self.benchmark_data:
            print("No benchmark data collected")
            return

        # Calculate statistics
        cpu_percent = [m['cpu_percent'] for m in self.benchmark_data]
        memory_percent = [m['memory_percent'] for m in self.benchmark_data]
        simulation_rtf = [m['simulation_rtf'] for m in self.benchmark_data if m['simulation_rtf'] > 0]

        # CPU statistics
        print(f"CPU Usage:")
        print(f"  Average: {np.mean(cpu_percent):.2f}%")
        print(f"  Peak: {np.max(cpu_percent):.2f}%")
        print(f"  Min: {np.min(cpu_percent):.2f}%")

        # Memory statistics
        print(f"\nMemory Usage:")
        print(f"  Average: {np.mean(memory_percent):.2f}%")
        print(f"  Peak: {np.max(memory_percent):.2f}%")
        print(f"  Average Used: {np.mean([m['memory_used_mb'] for m in self.benchmark_data]):.2f} MB")

        # Simulation RTF statistics (if available)
        if simulation_rtf:
            print(f"\nSimulation Real-Time Factor:")
            print(f"  Average: {np.mean(simulation_rtf):.3f}")
            print(f"  Min: {np.min(simulation_rtf):.3f}")
            print(f"  Max: {np.max(simulation_rtf):.3f}")

        # Topic rate statistics
        print(f"\nTopic Message Rates:")
        for topic in self.config['topics_to_monitor']:
            if topic in self.metrics['message_rates']:
                rates = [r for r in self.metrics['message_rates'][topic] if r > 0]
                if rates:
                    print(f"  {topic}: avg={np.mean(rates):.2f}Hz, max={np.max(rates):.2f}Hz")

        # Check against thresholds
        print(f"\nPERFORMANCE ALERTS:")
        thresholds = self.config['performance_thresholds']

        if np.max(cpu_percent) > thresholds['cpu_critical']:
            print(f"  CRITICAL: CPU usage exceeded {thresholds['cpu_critical']}%")
        elif np.max(cpu_percent) > thresholds['cpu_warning']:
            print(f"  WARNING: CPU usage exceeded {thresholds['cpu_warning']}%")

        if np.max(memory_percent) > thresholds['memory_critical']:
            print(f"  CRITICAL: Memory usage exceeded {thresholds['memory_critical']}%")
        elif np.max(memory_percent) > thresholds['memory_warning']:
            print(f"  WARNING: Memory usage exceeded {thresholds['memory_warning']}%")

        if simulation_rtf and np.mean(simulation_rtf) < thresholds['rtf_critical']:
            print(f"  CRITICAL: Average RTF below {thresholds['rtf_critical']}")
        elif simulation_rtf and np.mean(simulation_rtf) < thresholds['rtf_warning']:
            print(f"  WARNING: Average RTF below {thresholds['rtf_warning']}")

    def generate_report(self, output_file=None):
        """Generate detailed performance report"""
        if not output_file:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_file = f"performance_benchmark_{timestamp}.json"

        report = {
            'benchmark_info': {
                'start_time': self.start_time,
                'duration': self.duration,
                'config_file': self.config_file,
                'timestamp': datetime.now().isoformat()
            },
            'metrics': self.metrics,
            'summary': {
                'total_measurements': len(self.benchmark_data),
                'cpu_avg': np.mean([m['cpu_percent'] for m in self.benchmark_data]),
                'cpu_max': np.max([m['cpu_percent'] for m in self.benchmark_data]),
                'memory_avg': np.mean([m['memory_percent'] for m in self.benchmark_data]),
                'memory_max': np.max([m['memory_percent'] for m in self.benchmark_data])
            }
        }

        with open(output_file, 'w') as f:
            json.dump(report, f, indent=2, default=str)

        print(f"\nDetailed report saved to: {output_file}")
        return output_file

    def plot_results(self, output_file=None):
        """Generate plots of benchmark results"""
        if not output_file:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_file = f"performance_benchmark_{timestamp}.png"

        if not self.benchmark_data:
            print("No data to plot")
            return

        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('Digital Twin Performance Benchmark Results')

        timestamps = [m['timestamp'] - self.benchmark_data[0]['timestamp'] for m in self.benchmark_data]

        # CPU usage plot
        cpu_percent = [m['cpu_percent'] for m in self.benchmark_data]
        axes[0, 0].plot(timestamps, cpu_percent, label='CPU %', color='red')
        axes[0, 0].set_title('CPU Usage Over Time')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('CPU %')
        axes[0, 0].grid(True)
        axes[0, 0].axhline(y=80, color='orange', linestyle='--', label='Warning (80%)')
        axes[0, 0].axhline(y=90, color='red', linestyle='--', label='Critical (90%)')
        axes[0, 0].legend()

        # Memory usage plot
        memory_percent = [m['memory_percent'] for m in self.benchmark_data]
        axes[0, 1].plot(timestamps, memory_percent, label='Memory %', color='blue')
        axes[0, 1].set_title('Memory Usage Over Time')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Memory %')
        axes[0, 1].grid(True)
        axes[0, 1].axhline(y=80, color='orange', linestyle='--', label='Warning (80%)')
        axes[0, 1].axhline(y=90, color='red', linestyle='--', label='Critical (90%)')
        axes[0, 1].legend()

        # Simulation RTF plot (if available)
        simulation_rtf = [m['simulation_rtf'] for m in self.benchmark_data]
        valid_rtf = [r for r in simulation_rtf if r > 0]
        if valid_rtf:
            axes[1, 0].plot(timestamps[:len(valid_rtf)], valid_rtf, label='RTF', color='green')
            axes[1, 0].set_title('Simulation Real-Time Factor')
            axes[1, 0].set_xlabel('Time (s)')
            axes[1, 0].set_ylabel('RTF')
            axes[1, 0].grid(True)
            axes[1, 0].axhline(y=1.0, color='black', linestyle='--', label='Real-time (1.0)')
            axes[1, 0].axhline(y=0.8, color='orange', linestyle='--', label='Warning (0.8)')
            axes[1, 0].axhline(y=0.5, color='red', linestyle='--', label='Critical (0.5)')
            axes[1, 0].legend()
        else:
            axes[1, 0].text(0.5, 0.5, 'No RTF data available',
                           horizontalalignment='center', verticalalignment='center',
                           transform=axes[1, 0].transAxes)
            axes[1, 0].set_title('Simulation Real-Time Factor')

        # Topic rates plot
        if self.metrics['message_rates']:
            ax = axes[1, 1]
            for topic, rates in self.metrics['message_rates'].items():
                if any(r > 0 for r in rates):
                    # Truncate rates to match timestamps if needed
                    plot_rates = rates[:len(timestamps)]
                    ax.plot(timestamps[:len(plot_rates)], plot_rates, label=topic.split('/')[-1])
            ax.set_title('Topic Message Rates')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Rate (Hz)')
            ax.grid(True)
            ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')

        plt.tight_layout()
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        plt.close()

        print(f"Performance plots saved to: {output_file}")
        return output_file

def main():
    parser = argparse.ArgumentParser(description='Performance Benchmark for Digital Twin Systems')
    parser.add_argument('--config', '-c', help='Configuration file for benchmark')
    parser.add_argument('--duration', '-d', type=int, help='Benchmark duration in seconds')
    parser.add_argument('--output', '-o', help='Output file for detailed report')
    parser.add_argument('--plot', action='store_true', help='Generate performance plots')

    args = parser.parse_args()

    benchmark = PerformanceBenchmark(config_file=args.config)

    # Override duration if specified
    if args.duration:
        benchmark.config['duration'] = args.duration

    # Run benchmark
    benchmark.start_benchmark()

    # Analyze results
    benchmark.analyze_results()

    # Generate report
    report_file = benchmark.generate_report(output_file=args.output)

    # Generate plots if requested
    if args.plot:
        plot_file = benchmark.plot_results()

    print(f"\nBenchmark completed. Report: {report_file}")

if __name__ == '__main__':
    main()