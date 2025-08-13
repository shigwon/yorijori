import { createRouter, createWebHistory } from 'vue-router'
import Login from '../components/pages/Login.vue'
import Administrator from '../components/pages/Administrator.vue'
import Log from '../components/pages/Log.vue'
import RobotStatus from '../components/pages/RobotStatus.vue'
import RobotPosition from '../components/pages/RobotPosition.vue'
import WebRTCTest from '../components/pages/WebRTCTest.vue'
import Likes from '../components/pages/Likes.vue'
import DashboardDetail from '../components/pages/DashboardDetial.vue'
import TestSSE from '../components/pages/TESTsse.vue'

// 환경에 따른 경로 설정
const isDevelopment = import.meta.env.DEV
const prefix = isDevelopment ? '' : '/admin'
const baseUrl = isDevelopment ? '/' : '/admin/'

const routes = [
  {
    path: `${prefix}`,
    name: 'Login',
    component: Login
  },
  {
    path: `${prefix}/main`,
    name: 'Administrator',
    component: Administrator
  },
  {
    path: `${prefix}/dashboard-detail`,
    name: 'DashboardDetail',
    component: DashboardDetail
  },
  {
    path: `${prefix}/log`,
    name: 'Log',
    component: Log
  },
  {
    path: `${prefix}/robot-status`,
    name: 'RobotStatus',
    component: RobotStatus
  },
  {
    path: `${prefix}/robot-position`,
    name: 'RobotPosition',
    component: RobotPosition
  },
  {
    path: `${prefix}/webrtc-test`,
    name: 'WebRTCTest',
    component: WebRTCTest
  },
  {
    path: `${prefix}/likes`,
    name: 'Likes',
    component: Likes
  },
  {
    path: `${prefix}/test-sse`,
    name: 'TestSSE',
    component: TestSSE
  }
]

const router = createRouter({
  history: createWebHistory(baseUrl),
  routes
})

export default router 