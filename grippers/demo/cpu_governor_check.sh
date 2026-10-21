# Warn when the CPU governor is not "performance". Sourced by the demo
# launchers; prints only, never runs sudo.
#
# WHY THIS IS WORTH A WARNING
# ---------------------------
# Both demos are CPU-bound, and measurably so. On the PhysX teleop, measured on
# an RTX A4000 laptop: Isaac at 518% CPU with the GPU at 13%, RTF 0.454, and
# /clock + /joint_states ticking at only 18.8 Hz wall. MoveIt Servo publishes on
# the WALL clock at 250 Hz, so at that tick rate its feedback is ~13 control
# periods stale and the arm moves in visible steps.
#
# The trap is that Ubuntu's "Performance" power mode does NOT set this. On
# intel_pstate, power-profiles-daemon steers energy_performance_preference and
# leaves scaling_governor at "powersave" -- which is the name of the dynamic
# scaling algorithm, not a cap. So the desktop can read Performance while the
# governor reads powersave, and Isaac warns about the latter.
#
# It also reverts on every reboot, which is why this is checked at launch rather
# than written down once in a setup doc.
_gov_file=/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
if [[ -r "$_gov_file" ]] && [[ "$(cat "$_gov_file")" != "performance" ]]; then
  echo ""
  echo "  ############################################################"
  echo "  #  WARNING: CPU governor is '$(cat "$_gov_file")', not 'performance'."
  echo "  #"
  echo "  #  These demos are CPU-bound; this costs real-time factor,"
  echo "  #  which shows up as staggered/stepped arm motion."
  echo "  #"
  echo "  #  Fix it (reverts on reboot):"
  echo "  #"
  echo "  #      sudo cpupower frequency-set -g performance"
  echo "  #"
  echo "  #  Ubuntu's 'Performance' power mode does NOT set this."
  echo "  ############################################################"
  echo ""
fi
unset _gov_file
