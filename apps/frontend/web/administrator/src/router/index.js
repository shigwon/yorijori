import { createRouter, createWebHistory } from 'vue-router'
import Login from '../components/pages/Login.vue'
import Log from '../components/pages/Log.vue'
import RobotStatus from '../components/pages/RobotStatus.vue'
import RobotPosition from '../components/pages/RobotPosition.vue'
import WebRTCTest from '../components/pages/WebRTCTest.vue'
import Likes from '../components/pages/Likes.vue'
import DashboardDetail from '../components/pages/DashboardDetial.vue'

const routes = [
  {
    path: '/',
    name: 'Login',
    component: Login
  },
  {
    path: '/dashboard-detail',
    name: 'DashboardDetail',
    component: DashboardDetail
  },
  {
    path: '/log',
    name: 'Log',
    component: Log
  },
  {
    path: '/robot-status',
    name: 'RobotStatus',
    component: RobotStatus
  },
  {
    path: '/robot-position',
    name: 'RobotPosition',
    component: RobotPosition
  },
  {
    path: '/webrtc-test',
    name: 'WebRTCTest',
    component: WebRTCTest
  },
  {
    path: '/likes',
    name: 'Likes',
    component: Likes
  }
]

const router = createRouter({
  history: createWebHistory(),
  routes
})

export default router 