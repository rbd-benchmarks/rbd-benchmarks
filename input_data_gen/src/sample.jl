struct Sample{T}
    q::JointSegmentedVector{T}
    v::JointSegmentedVector{T}
    v̇::JointSegmentedVector{T}
    τ::JointSegmentedVector{T}
end
