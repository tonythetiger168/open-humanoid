#!/usr/bin/env python3
"""OpenHumanoid v1.0 - Single Joint Validation Report Generator

Generates comprehensive HTML/PDF test report with:
  - Test summary and Pass/Fail status
  - All measurement data tables
  - Plots (friction curve, step response, Bode, thermal)
  - Specifications vs. actual performance comparison
"""
import json
import os
from datetime import datetime
from typing import Dict

class ValidationReport:
    def __init__(self, joint_name: str, joint_type: str):
        self.joint_name = joint_name
        self.joint_type = joint_type
        self.date = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.sections = []

    def add_section(self, title: str, data: Dict, pass_fail: bool):
        self.sections.append({
            "title": title,
            "data": data,
            "status": "PASS" if pass_fail else "FAIL"
        })

    def generate_html(self, output_path: str):
        """Generate HTML report."""
        html = f"""<!DOCTYPE html>
<html>
<head>
    <title>Single Joint Validation Report - {self.joint_name}</title>
    <style>
        body {{ font-family: Arial, sans-serif; margin: 40px; background: #f5f5f5; }}
        .header {{ background: #2196F3; color: white; padding: 20px; border-radius: 8px; }}
        .section {{ background: white; margin: 20px 0; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }}
        .pass {{ color: #4CAF50; font-weight: bold; }}
        .fail {{ color: #F44336; font-weight: bold; }}
        table {{ width: 100%; border-collapse: collapse; margin: 10px 0; }}
        th, td {{ padding: 10px; text-align: left; border-bottom: 1px solid #ddd; }}
        th {{ background: #f0f0f0; }}
        .plot {{ max-width: 100%; margin: 10px 0; border: 1px solid #ddd; }}
        .summary {{ font-size: 1.2em; padding: 15px; background: #e8f5e9; border-radius: 8px; }}
        .summary.fail {{ background: #ffebee; }}
    </style>
</head>
<body>
    <div class="header">
        <h1>OpenHumanoid v1.0 - Single Joint Validation Report</h1>
        <p>Joint: <strong>{self.joint_name}</strong> | Type: <strong>{self.joint_type}</strong></p>
        <p>Date: {self.date}</p>
    </div>
"""

        # Overall status
        all_pass = all(s["status"] == "PASS" for s in self.sections)
        status_class = "pass" if all_pass else "fail"
        html += f"""
    <div class="summary {'fail' if not all_pass else ''}">
        <h2>Overall Status: <span class="{status_class}">{'PASS' if all_pass else 'FAIL'}</span></h2>
        <p>Passed: {sum(1 for s in self.sections if s['status']=='PASS')}/{len(self.sections)} tests</p>
    </div>
"""

        # Sections
        for section in self.sections:
            html += f"""
    <div class="section">
        <h2>{section['title']} <span class="{'pass' if section['status']=='PASS' else 'fail'}">{section['status']}</span></h2>
        <table>
"""
            for key, value in section['data'].items():
                if isinstance(value, float):
                    value_str = f"{value:.4f}"
                elif isinstance(value, list) and len(value) > 5:
                    value_str = f"[{len(value)} items]"
                else:
                    value_str = str(value)
                html += f"            <tr><th>{key}</th><td>{value_str}</td></tr>\n"

            html += """        </table>
    </div>
"""

        # Plots
        plot_files = [
            ("Friction Curve", "data/friction_curve.png"),
            ("Step Response", "data/step_response.png"),
            ("Bode Plot", "data/bode_plot.png"),
        ]

        for title, path in plot_files:
            if os.path.exists(path):
                html += f"""
    <div class="section">
        <h2>{title}</h2>
        <img class="plot" src="{path}" alt="{title}">
    </div>
"""

        html += """
</body>
</html>
"""

        with open(output_path, "w") as f:
            f.write(html)
        print(f"Report saved to {output_path}")

    def generate_json(self, output_path: str):
        """Generate machine-readable JSON report."""
        report = {
            "joint_name": self.joint_name,
            "joint_type": self.joint_type,
            "date": self.date,
            "overall_status": "PASS" if all(s["status"] == "PASS" for s in self.sections) else "FAIL",
            "sections": self.sections
        }
        with open(output_path, "w") as f:
            json.dump(report, f, indent=2)
        print(f"JSON report saved to {output_path}")

# ============================================================
# Master test runner (runs all SJV tests sequentially)
# ============================================================
def run_full_validation(joint_name: str, joint_type: str, mock: bool = True):
    """Run complete single joint validation sequence."""
    print("=" * 70)
    print(f"Single Joint Validation: {joint_name} ({joint_type})")
    print("=" * 70)

    from odrive_interface import ODriveAxis, ODriveConfig
    from encoder_calibration import EncoderCalibrator, EncoderConfig
    from friction_identification import FrictionIdentifier, FrictionIDConfig
    from performance_test import PerformanceTester, PerfTestConfig

    # Setup
    axis = ODriveAxis(ODriveConfig())
    axis.connect()

    report = ValidationReport(joint_name, joint_type)

    # 1. Encoder Calibration
    print("\n>>> Running Encoder Calibration...")
    cal = EncoderCalibrator(axis, EncoderConfig())
    enc_results = cal.run_full_calibration()
    report.add_section("Encoder Calibration", {
        "direction_correct": enc_results["direction"]["direction_correct"],
        "zero_error_deg": enc_results["zero_point"]["zero_error_deg"],
        "resolution_deg": enc_results["resolution"]["resolution_deg"],
        "repeatability_std_deg": enc_results["repeatability"]["std_deg"],
    }, all(r.get("pass", True) for r in enc_results.values()))
    cal.save_results("data/encoder_calibration.json")

    # 2. Friction Identification
    print("\n>>> Running Friction Identification...")
    fric = FrictionIdentifier(axis, FrictionIDConfig())
    fric_results = fric.identify_all()
    report.add_section("Friction Identification", {
        "torque_constant_Kt": fric_results["torque_constant"]["Kt"],
        "coulomb_friction_Nm": fric_results["friction"]["tau_c"],
        "viscous_friction_Nm_per_rads": fric_results["friction"]["tau_v"],
        "backlash_deg": fric_results["backlash"]["mean_backlash_deg"],
    }, all(r.get("pass", True) for r in fric_results.values()))
    fric.plot_friction_curve("data/friction_curve.png")
    fric.save_results("data/friction_identification.json")

    # 3. Performance Tests
    print("\n>>> Running Performance Tests...")
    perf = PerformanceTester(axis, PerfTestConfig())
    perf_results = perf.run_all_tests()
    report.add_section("Performance Tests", {
        "rise_time_s": perf_results["step_response"]["metrics"]["rise_time"],
        "settling_time_s": perf_results["step_response"]["metrics"]["settling_time"],
        "overshoot_percent": perf_results["step_response"]["metrics"]["overshoot"],
        "max_velocity_rads": perf_results["max_velocity"]["max_velocity"],
        "max_torque_Nm": perf_results["max_torque"]["max_torque"],
        "max_temp_C": perf_results["thermal"]["max_temperature"],
    }, all(r.get("pass", True) for r in perf_results.values()))
    perf.plot_step_response("data/step_response.png")
    perf.plot_bode("data/bode_plot.png")
    perf.save_results("data/performance_test.json")

    # Cleanup
    axis.disconnect()

    # Generate reports
    report.generate_html(f"reports/{joint_name}_validation_report.html")
    report.generate_json(f"reports/{joint_name}_validation_report.json")

    print("\n" + "=" * 70)
    print("Validation Complete!")
    print("=" * 70)

if __name__ == "__main__":
    run_full_validation("l_hip_pitch", "hip_pitch", mock=True)
