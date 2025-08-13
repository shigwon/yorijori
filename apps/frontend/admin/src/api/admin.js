import axios from 'axios'

// 관리자 전용 API 요청을 위한 axios instance
const adminClient = axios.create({
  baseURL: '/api/admin',
  timeout: 1500,
  headers: {
    'Content-Type': 'application/json',
    // 헤더 작성 예시:
    // 'X-Admin-Role': 'super_admin',
    // 'X-Request-ID': 'unique-request-id',
    // 'X-Client-Version': '1.0.0'
  }
})

// 요청 인터셉터
adminClient.interceptors.request.use(
  (config) => {
    // 관리자 토큰이 있으면 헤더에 추가 (테스트 단계에서는 선택적)
    const adminToken = localStorage.getItem('adminToken')
    if (adminToken) {
      config.headers.Authorization = `Bearer ${adminToken}`
    } else {
      // 테스트 단계: 토큰이 없어도 요청 진행
      console.log('테스트 모드: 토큰 없이 API 요청')
    }
    
    // 관리자 권한 헤더 추가
    config.headers['X-Admin-Role'] = 'admin'
    
    console.log('Admin API 요청:', config.method?.toUpperCase(), config.url)
    return config
  },
  (error) => {
    console.error('Admin API 요청 오류:', error)
    return Promise.reject(error)
  }
)

// 응답 인터셉터
adminClient.interceptors.response.use(
  (response) => {
    console.log('Admin API 응답:', response.status, response.config.url)
    return response
  },
  (error) => {
    console.error('Admin API 응답 오류:', error.response?.status, error.response?.data)
    
    // 401 Unauthorized 처리 (테스트 단계에서는 토큰이 없어도 허용)
    if (error.response?.status === 401) {
      // 테스트 단계: 토큰이 없어도 로그인 페이지로 강제 이동하지 않음
      console.log('테스트 모드: 401 오류 발생했지만 토큰이 없어도 계속 진행')
      localStorage.removeItem('adminToken')
      // window.location.href = '/' // 테스트 단계에서는 주석 처리
    }
    
    // 403 Forbidden 처리
    if (error.response?.status === 403) {
      console.error('관리자 권한이 없습니다.')
    }
    
    return Promise.reject(error)
  }
)

export default adminClient 