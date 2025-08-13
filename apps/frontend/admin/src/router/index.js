import { createRouter, createWebHistory } from 'vue-router'
import Login from '../components/pages/Login.vue'
import Administrator from '../components/pages/Administrator.vue'
import Log from '../components/pages/Log.vue'
import RobotStatus from '../components/pages/RobotStatus.vue'
import RobotPosition from '../components/pages/RobotPosition.vue'
import WebRTCTest from '../components/pages/WebRTCTest.vue'
import Likes from '../components/pages/Likes.vue'
import DashboardDetail from '../components/pages/DashboardDetial.vue'

const routes = [
  {
    path: '/admin/',
    name: 'Login',
    component: Login
  },
  {
    path: '/admin/main',
    name: 'Administrator',
    component: Administrator
  },
  {
    path: '/admin/dashboard-detail',
    name: 'DashboardDetail',
    component: DashboardDetail
  },
  {
    path: '/admin/log',
    name: 'Log',
    component: Log
  },
  {
    path: '/admin/robot-status',
    name: 'RobotStatus',
    component: RobotStatus
  },
  {
    path: '/admin/robot-position',
    name: 'RobotPosition',
    component: RobotPosition
  },
  {
    path: '/admin/webrtc-test',
    name: 'WebRTCTest',
    component: WebRTCTest
  },
  {
    path: '/admin/likes',
    name: 'Likes',
    component: Likes
  }
]

const router = createRouter({
  history: createWebHistory(),
  routes
})

export default router 