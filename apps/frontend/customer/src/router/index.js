import { createRouter, createWebHistory } from 'vue-router'

// 컴포넌트들을 동적 import로 로드
const WelcomeScreen = () => import('../components/01_WelcomeScreen.vue')
const HowToUseScreen = () => import('../components/02_HowToUseScreen.vue')
const TermsAgreementScreen = () => import('../components/03_TermsAgreementScreen.vue')
const PhotoSelectionScreen = () => import('../components/04_PhotoSelectionScreen.vue')
const CameraCapture = () => import('../components/05_CameraCapture.vue')
const LocationSettingScreen = () => import('../components/07_LocationSettingScreen.vue')
const DeliveryTrackingScreen = () => import('../components/08_DeliveryTrackingScreen.vue')
const FoodCompartmentScreen = () => import('../components/10_FoodCompartmentScreen.vue')
const SurveyScreen = () => import('../components/11_SurveyScreen.vue')

const routes = [
  {
    path: '/',
    redirect: '/welcome'
  },
  {
    path: '/welcome',
    name: 'Welcome',
    component: WelcomeScreen,
    meta: { progress: 0 }
  },
  {
    path: '/how-to-use',
    name: 'HowToUse',
    component: HowToUseScreen,
    meta: { progress: 15 }
  },
  {
    path: '/terms-agreement',
    name: 'TermsAgreement',
    component: TermsAgreementScreen,
    meta: { progress: 30 }
  },
  {
    path: '/photo-selection',
    name: 'PhotoSelection',
    component: PhotoSelectionScreen,
    meta: { progress: 45 }
  },
  {
    path: '/camera-capture',
    name: 'CameraCapture',
    component: CameraCapture,
    meta: { progress: 60 }
  },
  {
    path: '/location-setting',
    name: 'LocationSetting',
    component: LocationSettingScreen,
    meta: { progress: 75 }
  },
  {
    path: '/delivery-tracking',
    name: 'DeliveryTracking',
    component: DeliveryTrackingScreen,
    meta: { progress: 90 }
  },
  {
    path: '/food-compartment',
    name: 'FoodCompartment',
    component: FoodCompartmentScreen,
    meta: { progress: 95 }
  },
  {
    path: '/survey',
    name: 'Survey',
    component: SurveyScreen,
    meta: { progress: 100 }
  }
]

const router = createRouter({
  history: createWebHistory('/customer/'),
  routes
})

export default router
