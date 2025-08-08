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
    path: '/customer/',
    redirect: '/customer/welcome'
  },
  {
    path: '/customer/welcome',
    name: 'Welcome',
    component: WelcomeScreen,
    meta: { progress: 0 }
  },
  {
    path: '/customer/how-to-use',
    name: 'HowToUse',
    component: HowToUseScreen,
    meta: { progress: 15 }
  },
  {
    path: '/customer/terms-agreement',
    name: 'TermsAgreement',
    component: TermsAgreementScreen,
    meta: { progress: 30 }
  },
  {
    path: '/customer/photo-selection',
    name: 'PhotoSelection',
    component: PhotoSelectionScreen,
    meta: { progress: 45 }
  },
  {
    path: '/customer/camera-capture',
    name: 'CameraCapture',
    component: CameraCapture,
    meta: { progress: 60 }
  },
  {
    path: '/customer/location-setting',
    name: 'LocationSetting',
    component: LocationSettingScreen,
    meta: { progress: 75 }
  },
  {
    path: '/customer/delivery-tracking',
    name: 'DeliveryTracking',
    component: DeliveryTrackingScreen,
    meta: { progress: 90 }
  },
  {
    path: '/customer/food-compartment',
    name: 'FoodCompartment',
    component: FoodCompartmentScreen,
    meta: { progress: 95 }
  },
  {
    path: '/customer/survey',
    name: 'Survey',
    component: SurveyScreen,
    meta: { progress: 100 },
    props: (route) => ({ orderCode: route.query.code })
  }
]

const router = createRouter({
  history: createWebHistory(),
  routes
})

export default router
