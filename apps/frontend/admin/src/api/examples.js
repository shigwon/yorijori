import apiClient from './apiconfig.js'
import adminClient from './admin.js'

// ===== 일반 API 사용 예시 =====

// 로그인
export const login = async (email, password) => {
  try {
    const response = await apiClient.post('/admin/login', {
      email,
      password
    })
    return response.data
  } catch (error) {
    throw error
  }
}

// 로그아웃
export const logout = async () => {
  try {
    const response = await apiClient.post('/admin/logout')
    return response.data
  } catch (error) {
    throw error
  }
}

// ===== 관리자 API 사용 예시 =====

// 일일 주문 수 조회
export const getDailyOrderCount = async (date) => {
  try {
    const response = await adminClient.get(`/orders/count/daily?date=${date}`)
    return response.data
  } catch (error) {
    throw error
  }
}

// 주간 주문 수 조회
export const getWeeklyOrderCount = async (startDate) => {
  try {
    const response = await adminClient.get(`/orders/count/weekly?startDate=${startDate}`)
    return response.data
  } catch (error) {
    throw error
  }
}

// 월간 주문 수 조회
export const getMonthlyOrderCount = async (year, month) => {
  try {
    const response = await adminClient.get(`/orders/count/monthly?year=${year}&month=${month}`)
    return response.data
  } catch (error) {
    throw error
  }
}

// 시간별 주문 통계
export const getHourlyOrderStats = async (date) => {
  try {
    const response = await adminClient.get(`/orders/statistics/hourly?date=${date}`)
    return response.data
  } catch (error) {
    throw error
  }
}

// 주간 주문 통계
export const getWeeklyOrderStats = async (startDate) => {
  try {
    const response = await adminClient.get(`/orders/statistics/weekly?startDate=${startDate}`)
    return response.data
  } catch (error) {
    throw error
  }
}

// 기간별 주문 수 조회
export const getPeriodOrderCount = async (startDate, endDate) => {
  try {
    const response = await adminClient.get(`/orders/count/period?startDate=${startDate}&endDate=${endDate}`)
    return response.data
  } catch (error) {
    throw error
  }
}

// 로봇 목록 조회
export const getRobots = async () => {
  try {
    const response = await adminClient.get('/robots')
    return response.data
  } catch (error) {
    throw error
  }
}

// 특정 로봇 조회
export const getRobot = async (robotId) => {
  try {
    const response = await adminClient.get(`/robots/${robotId}`)
    return response.data
  } catch (error) {
    throw error
  }
}

// 위치 정보가 포함된 로봇 목록
export const getRobotsWithLocation = async () => {
  try {
    const response = await adminClient.get('/robots/with-location')
    return response.data
  } catch (error) {
    throw error
  }
}

// 리뷰 목록 조회
export const getReviews = async () => {
  try {
    const response = await adminClient.get('/reviews')
    return response.data
  } catch (error) {
    throw error
  }
}

// 특정 리뷰 조회
export const getReview = async (reviewId) => {
  try {
    const response = await adminClient.get(`/reviews/${reviewId}`)
    return response.data
  } catch (error) {
    throw error
  }
}

// 리뷰 평점 통계
export const getReviewRating = async () => {
  try {
    const response = await adminClient.get('/reviews/rating')
    return response.data
  } catch (error) {
    throw error
  }
}

// 스트림 관련
export const getStream = async () => {
  try {
    const response = await adminClient.get('/stream')
    return response.data
  } catch (error) {
    throw error
  }
}

// 로그 조회
export const getLogs = async () => {
  try {
    const response = await adminClient.get('/log')
    return response.data
  } catch (error) {
    throw error
  }
}

// 채팅 목록 조회
export const getAllChats = async () => {
  try {
    const response = await adminClient.get('/chat/all')
    return response.data
  } catch (error) {
    throw error
  }
}

// 특정 채팅 조회
export const getChat = async (chatId) => {
  try {
    const response = await adminClient.get(`/chat?id=${chatId}`)
    return response.data
  } catch (error) {
    throw error
  }
} 