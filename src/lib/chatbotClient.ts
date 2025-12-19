// Frontend API client for chatbot functionality
export interface ChatRequest {
  message: string;
  selected_text?: string;
}

export interface ChatResponse {
  response: string;
}

export interface EmbeddingResponse {
  embeddings: number[];
}

class ChatbotClient {
  private baseUrl: string;

  constructor() {
    // Use the backend URL from environment or default to local backend
    this.baseUrl = process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000';
  }
  
  async sendMessage(message: string, selectedText?: string): Promise<ChatResponse> {
    try {
      const response = await fetch(`${this.baseUrl}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message,
          selected_text: selectedText
        }),
      });
      
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      
      return await response.json();
    } catch (error) {
      console.error('Error sending message to chatbot:', error);
      throw error;
    }
  }
  
  async generateEmbedding(text: string): Promise<EmbeddingResponse> {
    try {
      const response = await fetch(`${this.baseUrl}/embeddings`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ text }),
      });
      
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      
      return await response.json();
    } catch (error) {
      console.error('Error generating embedding:', error);
      throw error;
    }
  }
}

export const chatbotClient = new ChatbotClient();