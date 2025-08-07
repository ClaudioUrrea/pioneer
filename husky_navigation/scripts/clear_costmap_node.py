#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty

class CostmapCleaner:
    def __init__(self):
        rospy.init_node("costmap_cleaner")
        self.clear_service = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)

        rospy.wait_for_service("/move_base/clear_costmaps")
        rospy.loginfo("🧹 Servicio de limpieza de costmaps disponible.")

        # Llama al servicio cada 10 segundos (ajusta si quieres)
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_callback)

    def timer_callback(self, event):
        try:
            self.clear_service()
            #rospy.loginfo("✅ Costmaps limpiados.")
        except rospy.ServiceException as e:
            rospy.logwarn(f"❌ Error al limpiar costmaps: {e}")

if __name__ == "__main__":
    try:
        CostmapCleaner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

